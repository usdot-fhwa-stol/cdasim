/*
 * Copyright (C) 2024 LEIDOS.
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

package org.eclipse.mosaic.interactions.application;

import org.apache.commons.lang3.builder.EqualsBuilder;
import org.apache.commons.lang3.builder.HashCodeBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import static org.apache.commons.lang3.builder.ToStringStyle.SHORT_PREFIX_STYLE;
import org.eclipse.mosaic.rti.api.Interaction;

public final class MsgerRequestTrafficEvent extends Interaction{
    
    private static final long serialVersionUID = 1L;
    private String vehicleId;
    private String parameterName;
    /**
     * String identifying the type of this interaction.
     */
    public final static String TYPE_ID = createTypeIdentifier(MsgerRequestTrafficEvent.class);
    
    public MsgerRequestTrafficEvent(long time, String vehicleId, String parameterName){
        super(time);
        this.vehicleId = vehicleId;
        this.parameterName = parameterName;
    }

    public String vehicleId(){
        return this.vehicleId;
    }

    public String getParameterName(){
        return this.parameterName;
    }

    @Override
    public int hashCode() {
        return new HashCodeBuilder(3, 23)
                .append(vehicleId)
                .append(parameterName)
                .toHashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (obj == this) {
            return true;
        }
        if (obj.getClass() != getClass()) {
            return false;
        }

        MsgerRequestTrafficEvent other = (MsgerRequestTrafficEvent) obj;
        return new EqualsBuilder()
                .append(this.vehicleId, other.vehicleId)
                .append(this.parameterName, other.parameterName)
                .isEquals();
    }

    @Override
    public String toString() {
        return new ToStringBuilder(this, SHORT_PREFIX_STYLE)
                .appendSuper(super.toString())
                .append("vehicleId", vehicleId)
                .append("Parameter", parameterName)
                .toString();
    }

}

