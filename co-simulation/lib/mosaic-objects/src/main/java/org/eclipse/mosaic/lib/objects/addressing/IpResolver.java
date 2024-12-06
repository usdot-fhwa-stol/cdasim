/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contact: mosaic@fokus.fraunhofer.de
 */

package org.eclipse.mosaic.lib.objects.addressing;

import java.net.Inet4Address;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import javax.annotation.Nonnull;

import org.eclipse.mosaic.rti.config.CIpResolver;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

/**
 * Class providing a singleton for global IP address management.
 */
public final class IpResolver {

    enum UnitType {
        VEHICLE("veh"),

        ROAD_SIDE_UNIT("rsu"),

        TRAFFIC_MANAGEMENT_CENTER("tmc"),

        TRAFFIC_LIGHT("tl"),

        CHARGING_STATION("cs"),

        CARLA_VEHICLE("carla"),

        CARMA_VEHICLE("carma"),

        MESSENGER_VEHICLE("msger"),

        SERVER("server");

        private String prefix;

        UnitType(String prefix) {
            this.prefix = prefix;
        }
    }

    private static IpResolver singleton;

    private final Inet4Address netMask;

    private Map<UnitType, Inet4Address> unitNetworks = new HashMap<>();

    private final int maxRange;

    private final BiMap<String, Inet4Address> addressMap = HashBiMap.create();

    /**
     * Sets the IPResolver singleton. This can only be done once.
     *
     * @param ipResolver the new instance of IPResolver
     */
    public static void setSingleton(IpResolver ipResolver) {
        if (singleton != null) {
            throw new RuntimeException("Can not set IpResolver twice");
        }
        singleton = ipResolver;
    }

    /**
     * Returns the {@link IpResolver} singleton.
     */
    public static IpResolver getSingleton() {
        if (singleton == null) {
            throw new RuntimeException("IpResolver is not set yet");
        }
        return singleton;
    }

    /**
     * Gets a configuration and performs basic plausibility checks apaches
     * commons-net could be helpful here.
     *
     * @param configuration configuration taken from json
     */
    public IpResolver(CIpResolver configuration) {
        Objects.requireNonNull(configuration.netMask, "Invalid IpResolver configuration: No netMask given");

        Objects.requireNonNull(configuration.vehicleNet, "Invalid IpResolver configuration: No vehicleNet given");

        Objects.requireNonNull(configuration.carlaVehicleNet,
                "Invalid IpResolver configuration: No carlaVehicleNet given");

        Objects.requireNonNull(configuration.carmaVehicleNet,
                "Invalid IpResolver configuration: No carmaVehicleNet given");

        Objects.requireNonNull(configuration.rsuNet, "Invalid IpResolver configuration: No rsuNet given");

        Objects.requireNonNull(configuration.tmcNet, "Invalid IpResolver configuration: No tmcNet given");

        Objects.requireNonNull(configuration.tlNet, "Invalid IpResolver configuration: No tlNet given");

        Objects.requireNonNull(configuration.csNet, "Invalid IpResolver configuration: No csNet given");

        Objects.requireNonNull(configuration.serverNet, "Invalid IpResolver configuration: No serverNet given");

        try {
            this.netMask = (Inet4Address) Inet4Address.getByName(configuration.netMask);

            unitNetworks.put(UnitType.VEHICLE, (Inet4Address) Inet4Address.getByName(configuration.vehicleNet));

            unitNetworks.put(UnitType.CARLA_VEHICLE,
                    (Inet4Address) Inet4Address.getByName(configuration.carlaVehicleNet));

            unitNetworks.put(UnitType.CARMA_VEHICLE,
                    (Inet4Address) Inet4Address.getByName(configuration.carmaVehicleNet));
            
            unitNetworks.put(UnitType.MESSENGER_VEHICLE, (Inet4Address) Inet4Address.getByName(configuration.msgerVehicleNet));

            unitNetworks.put(UnitType.ROAD_SIDE_UNIT, (Inet4Address) Inet4Address.getByName(configuration.rsuNet));

            unitNetworks.put(UnitType.TRAFFIC_MANAGEMENT_CENTER,
                    (Inet4Address) Inet4Address.getByName(configuration.tmcNet));

            unitNetworks.put(UnitType.TRAFFIC_LIGHT, (Inet4Address) Inet4Address.getByName(configuration.tlNet));

            unitNetworks.put(UnitType.CHARGING_STATION, (Inet4Address) Inet4Address.getByName(configuration.csNet));

            unitNetworks.put(UnitType.SERVER, (Inet4Address) Inet4Address.getByName(configuration.serverNet));

        } catch (UnknownHostException ex) {
            throw new RuntimeException("Could not parse IP addresses from configuration");
        }
        int flatMask = addressArrayToFlat(netMask.getAddress());
        maxRange = (~flatMask) - 1;

        for (UnitType type : UnitType.values()) {
            Inet4Address network = unitNetworks.get(type);
            if (network == null) {
                throw new IllegalArgumentException("Missing subnet configuration for unit type " + type.prefix);
            }
            int flatNet = addressArrayToFlat(network.getAddress());
            if ((flatNet & ~flatMask) != 0) {
                throw new IllegalArgumentException("Invalid subnet configuration for unit type " + type.prefix);
            }
        }
    }

    /**
     * returns the maxumum number of units that can be assigned IPs.
     *
     * @return the maximum range as an int
     */
    public int getMaxRange() {
        return maxRange;
    }

    /**
     * Returns the netmask for subnets.
     *
     * @return the netmask as a Inet4Address
     */
    public Inet4Address getNetMask() {
        return netMask;
    }

    /**
     * @param hostname the units ID veh_0; rsu_1; tl_2; cs_3 etc.
     * @return the hosts Inet4Address if it is registered or null if not
     */
    public Inet4Address lookup(String hostname) {
        return hostname != null ? addressMap.get(hostname) : null;
    }

    /**
     * @param address the Inet4Address to lookup
     * @return name of the host belonging to the given address or null if none found
     */
    public String reverseLookup(Inet4Address address) {
        return address != null ? addressMap.inverse().get(address) : null;
    }

    /**
     * Adds a new entry to the map.
     *
     * @param hostname hostname to register
     * @return returns the assigned IPv4 address
     */
    public Inet4Address registerHost(String hostname) {
        Inet4Address result = this.lookup(hostname);
        if (result == null) {
            result = nameToIp(hostname);
            addressMap.put(hostname, result);
        }
        return result;
    }

    /**
     * Converts a IPv4 address array to an integer.
     *
     * @param address the address as a byte array
     * @return the corresponding integer
     */
    int addressArrayToFlat(byte[] address) {
        if (address.length != 4) {
            throw new RuntimeException("Given address array is not 32 bit wide");
        }
        return ByteBuffer.wrap(address).getInt();
    }

    /**
     * Converts an integer to an byte array.
     *
     * @param address the address as integer
     * @return the byte array
     */
    byte[] addressFlatToArray(int address) {
        return ByteBuffer.allocate(4).putInt(address).array();
    }

    /**
     * Converts an IPv4 address to a hostname.
     *
     * @param address address that shall be converted to a hostname
     * @return hostname
     */
    public String ipToName(@Nonnull Inet4Address address) {
        int ip = addressArrayToFlat(address.getAddress());

        int netPart = addressArrayToFlat(netMask.getAddress());

        int client = ip & ~netPart;

        netPart = netPart & ip;

        for (Map.Entry<UnitType, Inet4Address> unitNetworkEntry : unitNetworks.entrySet()) {
            if (netPart == addressArrayToFlat(unitNetworkEntry.getValue().getAddress())) {
                return unitNetworkEntry.getKey().prefix + "_" + client;
            }
        }
        throw new RuntimeException("Unresolvable address " + address);
    }

    /**
     * gets an id and calculates the corresponding IP address.
     *
     * @param name the name which should be converted to an address
     * @return the corresponding address
     */
    public Inet4Address nameToIp(String name) {
        final int firstUnderscorePosition = name.indexOf('_');

        final int unitNumber = Integer.parseInt(name.substring(firstUnderscorePosition + 1));

        if (unitNumber > singleton.maxRange) {
            throw new IllegalArgumentException("IPv4 address exhausted");
        }

        String unitPrefix = name.substring(0, firstUnderscorePosition);

        for (Map.Entry<UnitType, Inet4Address> unitNetworkEntry : unitNetworks.entrySet()) {
            if (unitNetworkEntry.getKey().prefix.equals(unitPrefix)) {
                int network = addressArrayToFlat(unitNetworkEntry.getValue().getAddress());

                final Inet4Address ipResult;

                try {
                    ipResult = (Inet4Address) Inet4Address.getByAddress(addressFlatToArray(network | unitNumber));
                } catch (UnknownHostException ex) {
                    throw new RuntimeException("Error converting hostname to IP, address is not IPv4");
                }
                return ipResult;
            }
        }

        throw new RuntimeException("Erroneous hostname " + name);
    }
}