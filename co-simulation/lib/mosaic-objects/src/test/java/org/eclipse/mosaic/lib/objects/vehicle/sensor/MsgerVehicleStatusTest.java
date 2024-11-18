package org.eclipse.mosaic.lib.objects.vehicle.sensor;

import org.junit.Test;
import static org.junit.Assert.*;

import org.eclipse.mosaic.lib.objects.vehicle.MsgerVehicleStatus;
import org.eclipse.mosaic.lib.objects.vehicle.MsgerVehicleStatus.VehiclePose;
import org.eclipse.mosaic.lib.objects.vehicle.MsgerVehicleStatus.VehicleTwist;

public class MsgerVehicleStatusTest {

    @Test
    public void testVehiclePoseConstructorAndGetters() {
        VehiclePose pose = new MsgerVehicleStatus.VehiclePose(37.7749, -122.4194, 30.0);

        // Validate constructor and getters
        assertEquals(37.7749, pose.getLat(), 0.0001);
        assertEquals(-122.4194, pose.getLon(), 0.0001);
        assertEquals(30.0, pose.getAlt(), 0.0001);
    }

    @Test
    public void testVehiclePoseSetters() {
        VehiclePose pose = new MsgerVehicleStatus.VehiclePose(0.0, 0.0, 0.0);

        // Update values using setters
        pose.setLat(10.5);
        pose.setLon(20.5);
        pose.setAlt(5.0);

        // Validate updated values
        assertEquals(10.5, pose.getLat(), 0.0001);
        assertEquals(20.5, pose.getLon(), 0.0001);
        assertEquals(5.0, pose.getAlt(), 0.0001);
    }

    @Test
    public void testVehicleTwistConstructorAndGetters() {
        VehicleTwist twist = new MsgerVehicleStatus.VehicleTwist(1.0f, 2.0f, 3.0f);

        // Validate constructor and getters
        assertEquals(1.0f, twist.getX(), 0.0001);
        assertEquals(2.0f, twist.getY(), 0.0001);
        assertEquals(3.0f, twist.getZ(), 0.0001);
    }

    @Test
    public void testVehicleTwistSetters() {
        VehicleTwist twist = new MsgerVehicleStatus.VehicleTwist(0.0f, 0.0f, 0.0f);

        // Update values using setters
        twist.setX(10.5f);
        twist.setY(20.5f);
        twist.setZ(5.0f);

        // Validate updated values
        assertEquals(10.5f, twist.getX(), 0.0001);
        assertEquals(20.5f, twist.getY(), 0.0001);
        assertEquals(5.0f, twist.getZ(), 0.0001);
    }

    @Test
    public void testMsgerVehicleStatusConstructorAndGetters() {
        VehiclePose pose = new MsgerVehicleStatus.VehiclePose(37.7749, -122.4194, 30.0);
        VehicleTwist twist = new MsgerVehicleStatus.VehicleTwist(1.0f, 2.0f, 3.0f);
        MsgerVehicleStatus status = new MsgerVehicleStatus(pose, twist, true, false);

        // Validate constructor and getters
        assertEquals(pose, status.getVehiclePose());
        assertEquals(twist, status.getVehicleTwist());
        assertTrue(status.isSirenActive());
        assertFalse(status.isLightActive());
    }

    @Test
    public void testMsgerVehicleStatusSetters() {
        VehiclePose pose = new MsgerVehicleStatus.VehiclePose(0.0, 0.0, 0.0);
        VehicleTwist twist = new MsgerVehicleStatus.VehicleTwist(0.0f, 0.0f, 0.0f);
        MsgerVehicleStatus status = new MsgerVehicleStatus(pose, twist, false, false);

        // Update values using setters
        VehiclePose newPose = new MsgerVehicleStatus.VehiclePose(37.7749, -122.4194, 30.0);
        VehicleTwist newTwist = new MsgerVehicleStatus.VehicleTwist(1.0f, 2.0f, 3.0f);

        status.setVehiclePose(newPose);
        status.setVehicleTwist(newTwist);
        status.setSirenActive(true);
        status.setLightActive(true);

        // Validate updated values
        assertEquals(newPose, status.getVehiclePose());
        assertEquals(newTwist, status.getVehicleTwist());
        assertTrue(status.isSirenActive());
        assertTrue(status.isLightActive());
    }

    @Test
    public void testToString() {
        VehiclePose pose = new MsgerVehicleStatus.VehiclePose(37.7749, -122.4194, 30.0);
        VehicleTwist twist = new MsgerVehicleStatus.VehicleTwist(1.0f, 2.0f, 3.0f);
        MsgerVehicleStatus status = new MsgerVehicleStatus(pose, twist, true, false);

        // Verify toString output
        String toStringOutput = status.toString();
        assertTrue(toStringOutput.contains("VehiclePose [lat=37.7749, lon=-122.4194, alt=30.0]"));
        assertTrue(toStringOutput.contains("VehicleTwist [x=1.0, y=2.0, z=3.0]"));
        assertTrue(toStringOutput.contains("sirenActive=true"));
        assertTrue(toStringOutput.contains("lightActive=false"));
    }
}
