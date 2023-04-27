package gov.dot.fhwa.saxton;

import org.junit.Before;
import org.junit.Test;

public class CarmaV2xMessageTest {

    private final String sampleMessage = "Version=0.7\n" +
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

    @Before
    public void setUp() throws Exception {
        //manager = new InfrastructureInstanceManager();
        //registration = mock(InfrastructureRegistrationMessage.class);
    }

    @Test
    public void testCarmaV2xMessageParse() {
        CarmaV2xMessage test = new CarmaV2xMessage(sampleMessage.getBytes());
    }

}
