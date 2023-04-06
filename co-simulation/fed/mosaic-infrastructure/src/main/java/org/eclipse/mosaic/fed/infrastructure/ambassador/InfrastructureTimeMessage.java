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

import java.net.InetAddress;
import java.nio.charset.StandardCharsets;

/**
 * Message to be sent or received by the Infrastructure Device Adapter interface
 * NOTE: TODO See .ambassador for reference
 
 */
public class InfrastructureTimeMessage {
    private long timestep;
    private int seq;
    
 
    public InfrastructureTimeMessage() {
    }

    public long get_timestep()
    {
        return timestep;
    }
    public int get_seq()
    {
        return seq;
    }
    public void set_timestep(long new_timestep)
    {
        this.timestep = new_timestep;
    }
    public void set_seq(int new_seq)
    {
        this.seq = new_seq;
    }
    @Override
    public String toString() {
        return "InfrastructureTimeMessage [timestep=" + timestep + ", seq=" + seq + "]";
    }

}
