package org.eclipse.mosaic.fed.infrastructure.configuration;

import java.util.List;

import org.eclipse.mosaic.lib.geo.GeoPoint;

/**
 * Represents the configuration settings for a Road Side Unit (RSU) in a wireless communication infrastructure.
 */
public class RsuConfiguration {
    
    /**
     * The timestamp associated with the configuration, in nanoseconds.
     */
    public long time;
    
    /**
     * The name of the RSU.
     */
    public String name;
    
    /**
     * A group identifier for the RSU, used for grouping and organizing RSUs.
     */
    public String group;
    
    /**
     * A list of application names installed on the RSU.
     */
    public List<String> application;
    
    /**
     * The geographical position of the RSU on the map, represented as a GeoPoint object.
     */
    public GeoPoint position;
}
