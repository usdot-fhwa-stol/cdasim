/*
*Copyright (C) 2023 LEIDOS.
*
*Licensed under the Apache License, Version 2.0 (the "License"); you may not
*use this file except in compliance with the License. You may obtain a copy of
*the License at
*
*http://www.apache.org/licenses/LICENSE-2.0
*
*Unless required by applicable law or agreed to in writing, software
*distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
*WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
*License for the specific language governing permissions and limitations under
*the License.
*/
package org.eclipse.mosaic.fed.carla.carlaconnect;

import java.util.List;
import java.util.Map;

/**
 * Represents the state of an actor in CARLA.
 * This class provides type-safe access to actor state information
 * instead of using untyped Map&lt;String, Object&gt;.
 */
public class ActorState {
    
    /**
     * Represents the transform (position and rotation) of an actor.
     */
    public static class Transform {
        private final List<Double> location; // [x, y, z]
        private final List<Double> rotation; // [pitch, yaw, roll]
        
        public Transform(List<Double> location, List<Double> rotation) {
            this.location = location;
            this.rotation = rotation;
        }
        
        public List<Double> getLocation() {
            return location;
        }
        
        public List<Double> getRotation() {
            return rotation;
        }
    }
    
    /**
     * Represents the velocity information of an actor.
     */
    public static class Velocity {
        private final List<Double> linear; // [x, y, z]
        
        public Velocity(List<Double> linear) {
            this.linear = linear;
        }
        
        public List<Double> getLinear() {
            return linear;
        }
    }
    
    private final String type;
    private final Transform transform;
    private final Velocity velocity;
    private final String id;
    
    public ActorState(String type, Transform transform, Velocity velocity, String id) {
        this.type = type;
        this.transform = transform;
        this.velocity = velocity;
        this.id = id;
    }
    
    public String getType() {
        return type;
    }
    
    public Transform getTransform() {
        return transform;
    }
    
    public Velocity getVelocity() {
        return velocity;
    }
    
    public String getId() {
        return id;
    }
    
    /**
     * Creates an ActorState from a Map as returned by the XML-RPC server.
     * @param actorId The actor ID (key in the map)
     * @param stateMap The state map from the server
     * @return ActorState object, or null if conversion fails
     */
    @SuppressWarnings("unchecked")
    public static ActorState fromMap(String actorId, Map<String, Object> stateMap) {
        if (stateMap == null) {
            return null;
        }
        
        String type = null;
        if (stateMap.get("type") instanceof String) {
            type = (String) stateMap.get("type");
        }
        
        Transform transform = null;
        Object transformObj = stateMap.get("transform");
        if (transformObj instanceof Map) {
            Map<String, Object> transformMap = (Map<String, Object>) transformObj;
            Object locObj = transformMap.get("location");
            Object rotObj = transformMap.get("rotation");
            
            List<Double> location = null;
            List<Double> rotation = null;
            
            if (locObj instanceof List) {
                location = convertToList((List<?>) locObj);
            }
            if (rotObj instanceof List) {
                rotation = convertToList((List<?>) rotObj);
            }
            
            if (location != null && rotation != null) {
                transform = new Transform(location, rotation);
            }
        }
        
        Velocity velocity = null;
        Object velocityObj = stateMap.get("velocity");
        if (velocityObj instanceof Map) {
            Map<String, Object> velocityMap = (Map<String, Object>) velocityObj;
            Object linearObj = velocityMap.get("linear");
            
            if (linearObj instanceof List) {
                List<Double> linear = convertToList((List<?>) linearObj);
                if (linear != null) {
                    velocity = new Velocity(linear);
                }
            }
        }
        
        String id = actorId;
        if (stateMap.get("id") instanceof String) {
            id = (String) stateMap.get("id");
        }
        
        return new ActorState(type, transform, velocity, id);
    }
    
    /**
     * Converts a List of Number objects to List of Double.
     */
    private static List<Double> convertToList(List<?> list) {
        if (list == null) {
            return null;
        }
        
        List<Double> result = new java.util.ArrayList<>();
        for (Object item : list) {
            if (item instanceof Number) {
                result.add(((Number) item).doubleValue());
            } else {
                return null; // Invalid type
            }
        }
        return result;
    }
}

