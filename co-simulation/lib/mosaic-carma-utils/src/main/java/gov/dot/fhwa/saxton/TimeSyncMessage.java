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

package gov.dot.fhwa.saxton;

/**
 * Message to be sent or received by the Infrastructure Device Adapter interface
 * NOTE: TODO See .ambassador for reference
 * 
 */
public class TimeSyncMessage {
    // Timestamp in milliseconds
    private long timestep;
    // Sequence number for time message
    private int seq;

    public TimeSyncMessage() {
    }

    public long getTimestep() {
        return timestep;
    }

    public int getSeq() {
        return seq;
    }

    public void setTimestep(long newTimestep) {
        this.timestep = newTimestep;
    }

    public void setSeq(int newSeq) {
        this.seq = newSeq;
    }

    @Override
    public String toString() {
        return "InfrastructureTimeMessage [timestep=" + timestep + ", seq=" + seq + "]";
    }

}
