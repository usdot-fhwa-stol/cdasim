package org.eclipse.mosaic.fed.carma.ambassador;

import java.util.HashMap;
import java.util.Map;

public class CarmaInstanceManager {
    private Map<String, CarmaInstance>  managedInstances = new HashMap<>();
    private double currentSimulationTime;

    public void onV2XMessageTx(CarmaV2xMessage txMsg) {
        if (!managedInstances.containsKey(txMsg.getVehicleId())) {
            newCarmaInstance(
                txMsg.getVehicleId(), 
                txMsg.getVehiclePosX(), 
                txMsg.getVehiclePosY());
        } else {
            updateCarmaInstance(
                txMsg.getVehicleId(), 
                txMsg.getVehiclePosX(), 
                txMsg.getVehiclePosY());
        }
    }

    private void newCarmaInstance(String vehId, double xPos, double yPos) {
        managedInstances.put(vehId, new CarmaInstance(vehId, xPos, yPos, currentSimulationTime));

        // Instantiate NS-3 radio via AdHocCommunicationInteraction
    }

    private void updateCarmaInstance(String vehId, double xPos, double yPos) {
        managedInstances.get(vehId).setPosX(xPos);
        managedInstances.get(vehId).setPosY(yPos);
        managedInstances.get(vehId).setLastUpdateTime(currentSimulationTime);

        
    }

    private void moveCarmaInstanceRadio() {
        
    }
}
