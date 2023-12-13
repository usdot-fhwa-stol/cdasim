package gov.dot.fhwa.saxton;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

public class TimeSyncMessageTest {
     @Before
    public void setUp() throws Exception {
    }

    @Test
    public void testGetterSetterConstructor() {
        TimeSyncMessage msg = new TimeSyncMessage();
        msg.setSeq(100);
        msg.setTimestep(1200);
        assertEquals( 100, msg.getSeq());
        assertEquals(1200, msg.getTimestep());
    }
}
