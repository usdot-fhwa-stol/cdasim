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

package org.eclipse.mosaic.fed.application.ambassador;

import org.eclipse.mosaic.fed.application.ambassador.navigation.CentralNavigationComponent;
import org.eclipse.mosaic.fed.application.ambassador.simulation.AbstractSimulationUnit;
import org.eclipse.mosaic.fed.application.ambassador.util.FinishSimulationCallback;
import org.eclipse.mosaic.fed.application.config.CApplicationAmbassador;
import org.eclipse.mosaic.interactions.communication.V2xMessageRemoval;
import org.eclipse.mosaic.lib.math.RandomNumberGenerator;
import org.eclipse.mosaic.lib.objects.v2x.V2xMessage;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleRoute;
import org.eclipse.mosaic.lib.objects.vehicle.VehicleType;
import org.eclipse.mosaic.lib.util.objects.TimeCache;
import org.eclipse.mosaic.lib.util.scheduling.EventManager;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interactable;
import org.eclipse.mosaic.rti.api.InternalFederateException;

import edu.umd.cs.findbugs.annotations.SuppressWarnings;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.annotation.Nonnull;
import javax.annotation.Nullable;

public enum SimulationKernel implements FinishSimulationCallback {

    SimulationKernel;

    /**
     * The reference to send messages.
     */
    @SuppressWarnings(value = "SE_BAD_FIELD", justification = "The simulation kernel mustn't serializable.")
    @Nullable
    private Interactable interactable;

    private ClassLoader classLoader;
    private transient RandomNumberGenerator randomNumberGenerator;
    private final List<FinishSimulationCallback> finishSimulationCallbacks = new ArrayList<>();

    /**
     * Returns the list to register on a finish simulation event.
     *
     * @return the list.
     */
    public List<FinishSimulationCallback> getFinishSimulationCallbackList() {
        return finishSimulationCallbacks;
    }

    /**
     * The current simulation time. Unit: [ns].
     */
    private long currentSimulationTime;

    /**
     * The configuration for the application simulator.
     */
    @Nullable
    private CApplicationAmbassador configuration;


    /**
     * The configuration path for the application simulator.
     */
    @Nullable
    private File configurationPath;


    /**
     * The central navigation component in the application simulator.
     */
    transient CentralNavigationComponent navigation;

    /**
     * Map containing all the routes with the corresponding edge-id's.
     */
    private final Map<String, VehicleRoute> routes = new HashMap<>();

    /**
     * A view for the {@link #routes}.
     */
    private final Map<String, VehicleRoute> routesView = Collections.unmodifiableMap(routes);

    /**
     * Map containing all the names with the corresponding vehicle types.
     */
    private final Map<String, VehicleType> vehicleTypes = new HashMap<>();

    /**
     * The reference to add events.
     */
    @SuppressWarnings(value = "SE_BAD_FIELD", justification = "The simulation kernel mustn't be serializable.")
    @Nullable
    private EventManager eventManager;

    private final TimeCache<V2xMessage> v2XMessageCache = new TimeCache<>();

    /**
     * Get the {@link Interactable} reference.
     *
     * @return the reference to {@link Interactable}
     */
    public Interactable getInteractable() {
        if (interactable == null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_InteractableNotSet.toString());
        }
        return interactable;
    }

    /**
     * Set the {@link Interactable} reference.
     *
     * @param interactable The {@link Interactable} implementation to be set.
     */
    void setInteractable(Interactable interactable) {
        if (this.interactable != null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_InteractableAlreadySet.toString());
        }
        this.interactable = interactable;
    }

    public ClassLoader getClassLoader() {
        if (classLoader == null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_ClassLoaderNotSet.toString());
        }
        return classLoader;
    }

    public void setClassLoader(ClassLoader classLoader) {
        if (this.classLoader != null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_ClassLoaderAlreadySet.toString());
        }
        this.classLoader = classLoader;
    }

    public RandomNumberGenerator getRandomNumberGenerator() {
        if (randomNumberGenerator == null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_RandomNumberGeneratorNotSet.toString());
        }
        return randomNumberGenerator;
    }

    void setRandomNumberGenerator(RandomNumberGenerator randomNumberGenerator) {
        if (this.randomNumberGenerator != null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_RandomNumberGeneratorAlreadySet.toString());
        }
        this.randomNumberGenerator = randomNumberGenerator;
    }

    /**
     * Set the current simulation time.
     *
     * @param currentSimulationTime the current simulation time. Unit: [ns].
     */
    void setCurrentSimulationTime(long currentSimulationTime) {
        this.currentSimulationTime = currentSimulationTime;
    }

    /**
     * Returns the current simulation time.
     *
     * @return the current simulation time. Unit: [ns].
     */
    public long getCurrentSimulationTime() {
        return currentSimulationTime;
    }

    /**
     * Returns the simulator configuration.
     *
     * @return the simulator configuration.
     */
    public CApplicationAmbassador getConfiguration() {
        if (configuration == null) {
            throw new RuntimeException(ErrorRegister.SIMULATION_KERNEL_ConfigurationNotSet.toString());
        }
        return configuration;
    }

    /**
     * Set the simulator configuration.
     * This method should be only called once.
     *
     * @param configuration the simulator configuration.
     */
    public void setConfiguration(CApplicationAmbassador configuration) {
        if (this.configuration != null) {
            throw new RuntimeException(ErrorRegister.SIMULATION_KERNEL_ConfigurationAlreadySet.toString());
        }
        this.configuration = configuration;
    }

    /**
     * Returns the simulator configuration path.
     *
     * @return the simulator configuration path.
     */
    public File getConfigurationPath() {
        if (configurationPath == null) {
            throw new RuntimeException(ErrorRegister.SIMULATION_KERNEL_ConfigurationPathNotSet.toString());
        }
        return configurationPath;
    }

    /**
     * Set the simulator configuration path.
     * This method should be only called once.
     *
     * @param configurationPath the simulator configuration path.
     */
    public void setConfigurationPath(File configurationPath) {
        if (this.configurationPath != null) {
            throw new RuntimeException(ErrorRegister.SIMULATION_KERNEL_ConfigurationPathAlreadySet.toString());
        }
        this.configurationPath = configurationPath;
    }

    public void setCentralNavigationComponent(CentralNavigationComponent cnc) {
        if (this.navigation != null) {
            throw new RuntimeException(ErrorRegister.SIMULATION_KERNEL_CentralNavigationComponentAlreadySet.toString());
        }
        this.navigation = cnc;
    }

    /**
     * Get the reference to the central navigation component.
     *
     * @return CentralNavigationComponent
     */
    public CentralNavigationComponent getCentralNavigationComponent() {
        if (navigation == null) {
            throw new RuntimeException(ErrorRegister.SIMULATION_KERNEL_CentralNavigationComponentNotSet.toString());
        }
        return navigation;
    }

    /**
     * Returns a map containing all the Routes with the corresponding edge-id's.
     *
     * @return a map containing all the Routes with the corresponding edge-id's.
     */
    @Nonnull
    public Map<String, VehicleRoute> getRoutes() {
        return routes;
    }

    /**
     * Returns a view for the {@link #routes}.
     *
     * @return a view for the {@link #routes}.
     */
    @Nonnull
    public Map<String, VehicleRoute> getRoutesView() {
        return routesView;
    }

    /**
     * Returns a map containing all the names with the corresponding vehicle types.
     *
     * @return a map containing all the names with the corresponding vehicle types.
     */
    @Nonnull
    public Map<String, VehicleType> getVehicleTypes() {
        return vehicleTypes;
    }

    /**
     * Returns the cache for the v2x messages.
     *
     * @return the cache for the v2x messages.
     */
    public TimeCache<V2xMessage> getV2xMessageCache() {
        return v2XMessageCache;
    }

    /**
     * Get the {@link EventManager} reference.
     *
     * @return the reference to add events.
     */
    public EventManager getEventManager() {
        if (eventManager == null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_EventManagerNotSet.toString());
        }
        return eventManager;
    }

    /**
     * Set the {@link EventManager} reference.
     *
     * @param eventManager the manager to add events.
     */
    void setEventManager(EventManager eventManager) {
        if (this.eventManager != null) {
            throw new IllegalStateException(ErrorRegister.SIMULATION_KERNEL_EventManagerAlreadySet.toString());
        }
        this.eventManager = eventManager;
    }


    void garbageCollection() {
        if (interactable == null) {
            return;
        }
        // clean past environment events
        for (AbstractSimulationUnit simulationUnit : UnitSimulator.UnitSimulator.getAllUnits().values()) {
            simulationUnit.cleanPastEnvironmentEvents();
        }

        // is the garbage collection enabled?
        if (getConfiguration().messageCacheTime > 0) {
            // subtract the cache time from the current simulation time to get the last time an interaction should be cached
            long lastTime = SimulationKernel.getCurrentSimulationTime() - getConfiguration().messageCacheTime;
            if (lastTime > 0) {
                Set<Integer> removedIds = SimulationKernel.getV2xMessageCache().garbageCollection(lastTime);
                if (removedIds.isEmpty()) {
                    return;
                }

                try {
                    // notify all simulators about the V2xMessageRemoval interaction
                    getInteractable().triggerInteraction(
                            new V2xMessageRemoval(
                                    SimulationKernel.getCurrentSimulationTime(),
                                    removedIds
                            )
                    );
                } catch (IllegalValueException | InternalFederateException ex) {
                    throw new RuntimeException(ex);
                }
            }
        }
    }

    @Override
    public void finishSimulation() {
        for (FinishSimulationCallback cb : finishSimulationCallbacks) {
            cb.finishSimulation();
        }
    }
}
