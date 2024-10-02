/*
 * Copyright (C) 2024 LEIDOS.
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
    public void convertCartesianToGeographic() {

        String georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";

        MutableCartesianPoint cartesianOffset = new MutableCartesianPoint(200.00, 300.00, 0);
        GeoProjection transform = new Proj4Projection(GeoPoint.latLon(0.0, 0.0), cartesianOffset.getX(), cartesianOffset.getY(), georeference);

        MutableCartesianPoint testCartesianPoint = new MutableCartesianPoint(400.00, 600.00, 0);
        GeoPoint actualGeoPoint = transform.cartesianToGeographic(testCartesianPoint);

        assertEquals(actualGeoPoint.getLatitude(), 0.0027131084297879367, 0.0001d);
        assertEquals(actualGeoPoint.getLongitude(), 0.0017966305699434167, 0.0001d);
        assertEquals(actualGeoPoint.getAltitude(), 0.0, 0.0001d);

    }

    @Test
    public void convertGeographicToCartesian(){
        String georeference = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";
        GeoProjection transform = new Proj4Projection(GeoPoint.latLon(0.0, 0.0), 0.0,0.0, georeference);

        GeoPoint testGeoPoint = GeoPoint.latLon(0.000,0.000);

        CartesianPoint actualCartesian = transform.geographicToCartesian(testGeoPoint);
        assertEquals(actualCartesian.getX(), 0.0, 1d);
        assertEquals(actualCartesian.getY(), 0.0, 1d);
        assertEquals(actualCartesian.getZ(), 0.0, 1d);

    }


    @Test
    public void convertGeographictoUTM(){
        String georeference = "+proj=tmerc +lat_0=42.30059341574939 +lon_0=-83.69928318881136 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs";

        GeoProjection transform = new Proj4Projection(GeoPoint.latLon(42.30059341574939, -83.69928318881136), 0.0, 0.0, georeference);

        GeoPoint testGeoPoint = GeoPoint.latLon(38.9548994, -77.1481211);
        UtmZone zone = UtmZone.from(18,'n');
        UtmPoint actualUtmPoint = new MutableUtmPoint(313863.1656767028, 4313966.065972516, 0.0, zone);


        UtmPoint calculatedUtmPoint = transform.geographicToUtm(testGeoPoint);
        assertEquals(actualUtmPoint.getEasting(), calculatedUtmPoint.getEasting(), 1d);
        assertEquals(actualUtmPoint.getNorthing(), calculatedUtmPoint.getNorthing(), 1d);
        assertEquals(actualUtmPoint.getAltitude(), calculatedUtmPoint.getAltitude(), 1d);
        assertEquals(18, calculatedUtmPoint.getZone().number);

        //Test Special Zone Norway
        GeoPoint testGeoPoint2 = GeoPoint.latLon(77.8750, 20.9752);
        UtmPoint calculatedUtmPoint2 = transform.geographicToUtm(testGeoPoint2);
        assertEquals(33, calculatedUtmPoint2.getZone().number);

        //Test convert back to geographic
        GeoPoint calculatedGeoPoint = transform.utmToGeographic(calculatedUtmPoint);
        assertEquals(calculatedGeoPoint.getLatitude(), testGeoPoint.getLatitude(), 0.0001d);
        assertEquals(calculatedGeoPoint.getLongitude(),testGeoPoint.getLongitude(), 0.0001d);


    }

}