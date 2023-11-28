/*
 * Copyright 2023 Leidos
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package gov.dot.fhwa.saxton.fed.canneddetectedobject.ambassador;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.eclipse.mosaic.interactions.detector.DetectedObjectInteraction;
import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.math.Vector3d;
import org.eclipse.mosaic.lib.objects.detector.DetectedObject;
import org.eclipse.mosaic.lib.objects.detector.DetectionType;
import org.eclipse.mosaic.lib.objects.detector.Size;
import org.eclipse.mosaic.rti.api.AbstractFederateAmbassador;
import org.eclipse.mosaic.rti.api.IllegalValueException;
import org.eclipse.mosaic.rti.api.Interaction;
import org.eclipse.mosaic.rti.api.InternalFederateException;
import org.eclipse.mosaic.rti.api.parameters.AmbassadorParameter;

public class CannedDetectedObjectAmbassador extends AbstractFederateAmbassador {

    private ScheduledExecutorService executor;

    public CannedDetectedObjectAmbassador(AmbassadorParameter ambassadorParameter) {
        super(ambassadorParameter);

        executor = Executors.newSingleThreadScheduledExecutor();
    }

    @Override
    public void initialize(long startTime, long endTime) throws InternalFederateException {
        super.initialize(startTime, endTime);

        // executor.scheduleAtFixedRate(() -> publishDetectedObject(), 0, 1, TimeUnit.SECONDS);
    }

    @Override
    public synchronized void processTimeAdvanceGrant(long time) throws InternalFederateException {
        DetectedObject detection = new DetectedObject(
            DetectionType.CAR,
            1.0,
            "sensor-id",
            "proj-string",
            "object-id",
            CartesianPoint.xyz(1.0, 2.0, 3.0),
            new Vector3d(1.0, 2.0, 3.0),
            new Vector3d(1.0, 2.0, 3.0),
            new Size(1.0, 2.0, 3.0)
        );

        try {
            this.rti.triggerInteraction(new DetectedObjectInteraction(time, detection));
        } catch(IllegalValueException e) {
            log.error("Could not trigger interaction: ", e);
        }


        log.info("foo");
    }

    @Override
    public boolean isTimeConstrained() {
        return false;
    }

    @Override
    public boolean isTimeRegulating() {
        return true;
    }
}
