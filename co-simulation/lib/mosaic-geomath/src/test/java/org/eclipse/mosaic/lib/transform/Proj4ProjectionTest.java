/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contact: mosaic@fokus.fraunhofer.de
 */

package org.eclipse.mosaic.lib.transform;

import static org.junit.Assert.assertEquals;

import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.geo.MutableGeoPoint;
import org.eclipse.mosaic.lib.geo.MutableCartesianPoint;
import org.eclipse.mosaic.lib.geo.MutableUtmPoint;
import org.eclipse.mosaic.lib.geo.UtmPoint;
import org.eclipse.mosaic.lib.geo.UtmZone;
import org.eclipse.mosaic.lib.math.Vector3d;

import org.junit.Test;


public class Proj4ProjectionTest {
    @Test
    public void convert_cartesian_to_geographic() {

        String georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";

        MutableCartesianPoint cartesianOffset = new MutableCartesianPoint(200.00, 300.00, 0);
        GeoProjection transform = new Proj4Projection(GeoPoint.latLon(0.0, 0.0), cartesianOffset.getX(), cartesianOffset.getY(), georeference);

        MutableCartesianPoint testCartesianPoint = new MutableCartesianPoint(400.00, 600.00, 0);
        GeoPoint actualGeoPoint = transform.cartesianToGeographic(testCartesianPoint);

        assertEquals(actualGeoPoint.getLatitude(), 0.0027131084297879367, 0.0001d);
        assertEquals(actualGeoPoint.getLongitude(), 0.0017966305699434167, 0.0001d);
        assertEquals(actualGeoPoint.getAltitude(), 0.0, 0.0001d);

    }

    public void convert_geographic_to_cartesian(){
        String georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        GeoProjection transform = new Proj4Projection(GeoPoint.latLon(0.0, 0.0), 0.0,0.0, georeference);

        GeoPoint testGeoPoint = GeoPoint.latLon(0.0,0.0);

        CartesianPoint actualCartesian = transform.geographicToCartesian(testGeoPoint);
        assertEquals(actualCartesian.getX(), 0.0, 1d);
        assertEquals(actualCartesian.getY(), 0.0, 1d);
        assertEquals(actualCartesian.getZ(), 0.0, 1d);

    }


}