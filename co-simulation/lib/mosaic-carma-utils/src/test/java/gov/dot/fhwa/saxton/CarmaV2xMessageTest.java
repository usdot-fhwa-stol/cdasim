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

import org.apache.commons.codec.DecoderException;
import org.junit.Before;
import org.junit.Test;
import org.apache.commons.codec.binary.Hex;

public class CarmaV2xMessageTest {

    private final String sampleMessage =
            "Version=0.7\n" +
            "Type=BSM\n" +
            "PSID=0020\n" +
            "Priority=6\n" +
            "TxMode=ALT\n" +
            "TxChannel=172\n" +
            "TxInterval=0\n" +
            "DeliveryStart=\n" +
            "DeliveryStop=\n" +
            "Signature=False\n" +
            "Encryption=False\n" +
            "Payload=00142500400000000f0e35a4e900eb49d20000007fffffff8ffff080fdfa1fa1007fff8000960fa0\n";

    private final String sampleMessage2 = "450001039c69400040119f7e7f000001" +
                    "7f0000018fdc05ec00efff0256657273" +
                    "696f6e3d302e370a547970653d42534d" +
                    "0a505349443d303032300a5072696f72" +
                    "6974793d360a54784d6f64653d414c54" +
                    "0a54784368616e6e656c3d3137320a54" +
                    "78496e74657276616c3d300a44656c69" +
                    "7665727953746172743d0a44656c6976" +
                    "65727953746f703d0a5369676e617475" +
                    "72653d46616c73650a456e6372797074" +
                    "696f6e3d46616c73650a5061796c6f61" +
                    "643d3030313432353032633030303030" +
                    "30303239316562356134653930306562" +
                    "34396432303030303030376666666666" +
                    "66663866666666303830666466613166" +
                    "61313030376666663830303039363066" +
                    "61300a";

    @Before
    public void setUp() throws Exception {
    }

    @Test
    public void testCarmaV2xMessageParse() {
        CarmaV2xMessage test = new CarmaV2xMessage(sampleMessage.getBytes());
        assert(test != null);
    }

    @Test
    public void testCarmaV2xMessageParse2() {
        String sampleMessage2 = "jaksldfjkl;asdfjkl;asdfjkalsdfjkdjjsdjdf" + sampleMessage;
        CarmaV2xMessage test = new CarmaV2xMessage(sampleMessage2.getBytes());
        assert(test != null);
    }

    @Test
    public void testCarmaV2xMessageParse3() throws DecoderException {
        byte[] bytes = Hex.decodeHex(sampleMessage2.toCharArray());
        CarmaV2xMessage test = new CarmaV2xMessage(bytes);
        assert(test != null);
    }

}
