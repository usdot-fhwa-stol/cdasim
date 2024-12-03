/*
 * Copyright (C) 2019-2022 LEIDOS.
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

import java.net.InetAddress;
import java.util.Arrays;
import java.util.regex.Pattern;

/**
 * Message to be sent or received by the CARMA Platform NS-3 Adapater interface
 */
public class CarmaV2xMessage {
    private InetAddress originAddress;
    private String version;
    private String type;
    private int psid;
    private int priority;
    private String txMode;
    private int txChannel; 
    private int txInterval;
    private String deliveryStart;
    private String deliveryStop;
    private String payload;
    private boolean signature;
    private boolean encryption;

    public CarmaV2xMessage(InetAddress originAddress, String version, String type, int psid, String vehicleId, int priority, String txMode,
            int txChannel, int txInterval, String deliveryStart, String deliveryStop, boolean signature,
            boolean encryption) {
        this.originAddress = originAddress;
        this.version = version;
        this.type = type;
        this.psid = psid;
        this.priority = priority; this.txMode = txMode;
        this.txChannel = txChannel;
        this.txInterval = txInterval;
        this.deliveryStart = deliveryStart;
        this.deliveryStop = deliveryStop;
        this.signature = signature;
        this.encryption = encryption;
    }

    public CarmaV2xMessage(String version, String type, int psid, String vehicleId, int priority, String txMode,
                           int txChannel, int txInterval, String deliveryStart, String deliveryStop, boolean signature,
                           boolean encryption) {
        this.version = version;
        this.type = type;
        this.psid = psid;
        this.priority = priority; this.txMode = txMode;
        this.txChannel = txChannel;
        this.txInterval = txInterval;
        this.deliveryStart = deliveryStart;
        this.deliveryStop = deliveryStop;
        this.signature = signature;
        this.encryption = encryption;
    }

    public CarmaV2xMessage(byte[] bytes) {
        parseV2xMessage(bytes);
    }

    public String getVersion() {
        return version;
    }

    public String getType() {
        return type;
    }

    public int getPsid() {
        return psid;
    }

    public int getPriority() {
        return priority;
    }

    public String getTxMode() {
        return txMode;
    }

    public int getTxChannel() {
        return txChannel;
    }

    public int getTxInterval() {
        return txInterval;
    }

    public String getDeliveryStart() {
        return deliveryStart;
    }

    public String getDeliveryStop() {
        return deliveryStop;
    }

    public boolean isSignature() {
        return signature;
    }

    public boolean isEncryption() {
        return encryption;
    }

    /**
     * Constructor helper, decodes the format sent by the CARMA Platform driver
     * @param buf A binary array containing the data sent from the CARMA Platform's ns3 adapter
     */
    private void parseV2xMessage(byte[] buf)  {
        String rawMsg = new String(buf);
        String msg = rawMsg.substring(rawMsg.indexOf("Version"), rawMsg.length());

        // Modeled after Stackoverflow answer by user Eritrean: https://stackoverflow.com/users/5176992/eritrean
        // On Question: https://stackoverflow.com/questions/61029164/how-to-split-string-by-comma-and-newline-n-in-java
        // Asked by user: https://stackoverflow.com/users/12315939/kopite1905
        // Used under CC BY-SA 4.0 license: https://creativecommons.org/licenses/by-sa/4.0/
        String[] msgParts = Pattern.compile("\\R")
                .splitAsStream(msg)
                .map(s -> s.split("="))
                .flatMap(Arrays::stream)
                .toArray(String[]::new);
        
        for (int i = 0; i < msgParts.length; i++) {
            if (msgParts[i].equals("Version")) {
                version = msgParts[++i];
            } else if (msgParts[i].equals("Type")) {
                type = msgParts[++i];
            } else if (msgParts[i].equals("PSID")) {
                if (msgParts[i + 1].startsWith("0x")) {
                    psid = Integer.parseInt(msgParts[++i].substring(2), 16);
                } else {
                    psid = Integer.parseInt(msgParts[++i], 16);
                }
            } else if (msgParts[i].equals("Priority")) {
                priority = Integer.parseInt(msgParts[++i]);
            } else if (msgParts[i].equals("TxMode")) {
                txMode = msgParts[++i];
            } else if (msgParts[i].equals("TxChannel")) {
                txChannel = Integer.parseInt(msgParts[++i]);
            } else if (msgParts[i].equals("TxInterval")) {
                txInterval = Integer.parseInt(msgParts[++i]);
            } else if (msgParts[i].equals("DeliveryStart")) {
               // NOP 
            } else if (msgParts[i].equals("DeliveryStop")) {
               // NOP 
            } else if (msgParts[i].equals("Signature")) {
                signature = Boolean.parseBoolean(msgParts[++i]);
            } else if (msgParts[i].equals("Encryption")) {
                encryption = Boolean.parseBoolean(msgParts[++i]);
            } else if (msgParts[i].equals("Payload")) {
                payload = msgParts[++i];
                break; // Break on the final field of the message
            } else {
                throw new IllegalArgumentException("No such field in CarmaV2xMessage: " + msgParts[i]);
            }
        }
    }

    public InetAddress getOriginAddress() {
        return originAddress;
    }

    public String getPayload() {
        return payload;
    }

    public void setPayload(String payload) {
        this.payload = payload;
    }
}
