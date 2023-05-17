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

    private final String sampleMessage3 =
            "Version=0.7\n" +
                    "Type=BSM\n" +
                    "PSID=0x0020\n" +
                    "Priority=6\n" +
                    "TxMode=ALT\n" +
                    "TxChannel=172\n" +
                    "TxInterval=0\n" +
                    "DeliveryStart=\n" +
                    "DeliveryStop=\n" +
                    "Signature=False\n" +
                    "Encryption=False\n" +
                    "Payload=00142500400000000f0e35a4e900eb49d20000007fffffff8ffff080fdfa1fa1007fff8000960fa0\n";

    private static final String sampleMessage5 =
            "45000364c73f400040111842ac020001" +
            "ac020002977b05ed03505b6956657273" +
            "696f6e3d302e370a547970653d4d4150" +
            "0a505349443d3078383030320a507269" +
            "6f726974793d370a54784d6f64653d43" +
            "4f4e540a54784368616e6e656c3d3138" +
            "330a5478496e74657276616c3d300a44" +
            "656c697665727953746172743d0a4465" +
            "6c697665727953746f703d0a5369676e" +
            "61747572653d46616c73650a456e6372" +
            "797074696f6e3d46616c73650a506179" +
            "6c6f61643d3030313238313533333831" +
            "31333032303230344244413435344344" +
            "43463831343344344443343838313138" +
            "36303232343136343830323238303030" +
            "38303032323937443442433830413041" +
            "30413938323538323539323341393042" +
            "32463245343138393836463431423730" +
            "30363438303630353430333832413032" +
            "30313434303135343830303130303034" +
            "35323144394630303134313431363043" +
            "37433432413138373938353836313935" +
            "30324134324130363045393237313030" +
            "36363230303034303031303542453642" +
            "46343143384144454435383136454243" +
            "30353035303744434238363045433537" +
            "41454144353037394530323832383930" +
            "30383930303031303030343137323233" +
            "41353037323842373530463943364541" +
            "39453841453438304130413046363837" +
            "34364144343437433030323832383930" +
            "30413130383037303834303430343038" +
            "30334239303030323030303632423638" +
            "44353330354431463932363941373235" +
            "30323744383335324637323836374436" +
            "43383234303333343030303430303043" +
            "35334635423736314142424237443335" +
            "44334330383133454331413342414143" +
            "31364246433034383035304234303330" +
            "35413032303243343032323038303031" +
            "30303033313046453535463834394143" +
            "44363038443841434531333642343430" +
            "30303044464534383038383830303038" +
            "30303230383633363543303031374431" +
            "36313245423334303236303637343034" +
            "38393533393039303742443834383035" +
            "30323430333031323031433038303032" +
            "34303030323030303030303930303236" +
            "31383041304130463238353236303031" +
            "34303030313030303030303136394643" +
            "31353835424431444130303042303030" +
            "30383030303030304133424232463433" +
            "39343539413830303630303030343030" +
            "30303030343644353543343136433637" +
            "4634300a";

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

    @Test
    public void testCarmaV2xMessageParse4() throws DecoderException {
        CarmaV2xMessage test = new CarmaV2xMessage(sampleMessage3.getBytes());
        assert(test != null);
    }

    @Test
    public void testCarmaV2xMessageParse5() throws DecoderException {
        byte[] bytes = Hex.decodeHex(sampleMessage5.toCharArray());
        CarmaV2xMessage test = new CarmaV2xMessage(bytes);
        assert(test != null);
    }

}
