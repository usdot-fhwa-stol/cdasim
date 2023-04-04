/*
 * Copyright (C) 2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
 
package org.eclipse.mosaic.fed.infrastructure.ambassador;
import org.eclipse.mosaic.lib.geo.GeoPoint;

/**
 * A message to be sent by Infrastructure Device when it registers with the carma-mosaic ambassador
 * NOTE: TODO See carma.ambassador for reference
 */
public class InfrastructureRegistrationMessage {
    private String rxMessageIpAddress;
    private String intersectionId = null;
    private String infrastructureId;

    private int rxMessagePort = 1536;
    private int timeSyncPort = 1517; // TODO
    private GeoPoint location = null;

    public InfrastructureRegistrationMessage(String rxMessageIpAddress, String intersectionId, String infrastructureId,
                                            int rxMessagePort, int timeSyncPort, GeoPoint location) {
        this.rxMessageIpAddress = rxMessageIpAddress;
        this.intersectionId = intersectionId;
        this.infrastructureId = infrastructureId;
        this.rxMessagePort = rxMessagePort;
        this.timeSyncPort = timeSyncPort;
        this.location = location;
    }

    public String getRxMessageIpAddress(){
        return this.rxMessageIpAddress;
    }

    public String getIntersectionId(){
        return this.intersectionId;
    }

    public String getInfrastructureId(){
        return this.infrastructureId;
    }

    public int getRxMessagePort(){
        return this.rxMessagePort;
    }

    public int getTimeSyncPort(){
        return this.timeSyncPort;
    }

    public GeoPoint getLocation(){
        return this.location;
    }

    public void setRxMessageIpAddress(String rxMessageIpAddress){
        this.rxMessageIpAddress = rxMessageIpAddress;
    }

    public void setIntersectionId(String intersectionId){
        this.intersectionId = intersectionId;
    }

    public void setInfrastructureId(String infrastructureId){
        this.infrastructureId = infrastructureId;
    }

    public void setRxMessagePort(int rxMessagePort){
        this.rxMessagePort = rxMessagePort;
    }

    public void setTimeSyncPort(int timeSyncPort){
        this.timeSyncPort = timeSyncPort;
    }

    public void setLocation(GeoPoint location){
        this.location = location;
    }

    @Override
    public String toString() {
        return "InfrastructureRegistrationMessage [rxMessageIpAddress=" + rxMessageIpAddress + ", intersectionId=" + intersectionId
                + ", infrastructureId=" + infrastructureId + ", rxMessagePort=" + rxMessagePort 
                + ", timeSyncPort=" + timeSyncPort + ", location=" + location + "]";
    }
    

}
