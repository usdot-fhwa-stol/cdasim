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

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;


import org.eclipse.mosaic.lib.geo.CartesianPoint;
import org.eclipse.mosaic.lib.geo.GeoPoint;
import org.eclipse.mosaic.lib.geo.MutableCartesianPoint;
import org.eclipse.mosaic.lib.geo.MutableGeoPoint;
import org.eclipse.mosaic.lib.geo.MutableUtmPoint;
import org.eclipse.mosaic.lib.geo.UtmPoint;
import org.eclipse.mosaic.lib.geo.UtmZone;
import org.eclipse.mosaic.lib.math.MathUtils;
import org.eclipse.mosaic.lib.math.Vector3d;


import org.locationtech.proj4j.CRSFactory;
import org.locationtech.proj4j.CoordinateReferenceSystem;
import org.locationtech.proj4j.CoordinateTransform;
import org.locationtech.proj4j.CoordinateTransformFactory;
import org.locationtech.proj4j.ProjCoordinate;



public class Proj4Projection extends GeoProjection {

    // private final String georeference;
    private GeoPoint geoOrigin;
    UtmZone zone;

    private double x_offset;
    private double y_offset;

    private String wgs84ReferenceString;
    private String cartesianReferenceString;
    CoordinateReferenceSystem wgs84CRS;
    CoordinateReferenceSystem cartesianCRS;
    CoordinateReferenceSystem utmCRS;


    public Proj4Projection(GeoPoint origin, double x_offset, double y_offset, String georef){
        this.geoOrigin = origin;
        this.cartesianReferenceString = georef;
        this.wgs84ReferenceString = "EPSG:4326";
        this.x_offset = x_offset;
        this.y_offset = y_offset;


        CRSFactory crsFactory = new CRSFactory();
        this.cartesianCRS = crsFactory.createFromParameters("custom_proj", this.cartesianReferenceString);
        this.wgs84CRS = crsFactory.createFromName(wgs84ReferenceString);

        this.zone = getUTMZone(origin);
        String utm_proj_str;
        if (this.zone.isNorthernHemisphere())
        {
            utm_proj_str = "+proj=utm +zone=" + zone.number + " +north";
        }

        utm_proj_str = "+proj=utm +zone=" + zone.number + " +south";
        this.utmCRS = crsFactory.createFromParameters("custom_proj", utm_proj_str);

    }

    @Override
    public Vector3d geographicToVector(GeoPoint geographic, Vector3d result){
        // TODO: Update - required to be defined by the base class, but is non-essential here
        return (new Vector3d());
    }

    @Override
    public MutableGeoPoint vectorToGeographic(Vector3d vector3d, MutableGeoPoint result) {
        // TODO: Update - required to be defined by the base class, but is non-essential here
        return (new MutableGeoPoint());
    }

    @Override
    public MutableCartesianPoint geographicToCartesian(GeoPoint geographic, MutableCartesianPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(geographic.getLongitude(),geographic.getLatitude());
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(cartesianCRS, wgs84CRS);

        transform.transform(sourceCoord, targetCoord);
        MutableCartesianPoint output = new MutableCartesianPoint(targetCoord.x, targetCoord.y, 0.0);

        return (output);
    }

    @Override
    public MutableGeoPoint cartesianToGeographic(CartesianPoint cartesian, MutableGeoPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(cartesian.getX() - this.x_offset, cartesian.getY() - this.y_offset);
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(cartesianCRS, wgs84CRS);

        transform.transform(sourceCoord, targetCoord);
        //Transform returned as lat,lon = (targetCoord.y, targetCoord.x)
        MutableGeoPoint output = new MutableGeoPoint(targetCoord.y, targetCoord.x);

        return output;
    }

    @Override
    public Vector3d utmToVector(UtmPoint utm, Vector3d result) {
        // TODO: Update - required to be defined by the base class, but is non-essential here
        return geographicToVector(utmToGeographic(utm));
    }

    @Override
    public MutableUtmPoint vectorToUtm(Vector3d vector, MutableUtmPoint result) {
        // TODO: Update - required to be defined by the base class, but is non-essential here
        return geographicToUtm(vectorToGeographic(vector), result);
    }

    @Override
    public MutableUtmPoint geographicToUtm(GeoPoint geoPoint, MutableUtmPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(geoPoint.getLongitude(),geoPoint.getLatitude());
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(cartesianCRS, utmCRS);

        transform.transform(sourceCoord, targetCoord);
        MutableUtmPoint output = new MutableUtmPoint(targetCoord.x, targetCoord.y, 0.0, getUTMZone(geoPoint));

        return (output);
    }

    @Override
    public MutableGeoPoint utmToGeographic(UtmPoint utmPoint, MutableGeoPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(utmPoint.getEasting(), utmPoint.getNorthing());
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(utmCRS, wgs84CRS);

        transform.transform(sourceCoord, targetCoord);
        //Transform returned as lon,lat = (targetCoord.x, targetCoord.y)
        return new MutableGeoPoint(targetCoord.x, targetCoord.y);
    }

    public UtmZone getUTMZone(GeoPoint geoPoint){
        int zoneNumber;

        // Make sure the longitude is between -180.00 .. 179.9
        double longTemp = (geoPoint.getLongitude() + 180) - (int) ((geoPoint.getLongitude() + 180) / 360) * 360 - 180;

        zoneNumber = (int) ((longTemp + 180) / 6) + 1;

        if (geoPoint.getLatitude() >= 56.0 && geoPoint.getLatitude() < 64.0 && longTemp >= 3.0 && longTemp < 12.0) {
            zoneNumber = 32;
        }

        // Special zones for Svalbard
        if (geoPoint.getLatitude() >= 72.0 && geoPoint.getLatitude() < 84.0) {
            if (longTemp >= 0.0 && longTemp < 9.0) {
                zoneNumber = 31;
            } else if (longTemp >= 9.0 && longTemp < 21.0) {
                zoneNumber = 33;
            } else if (longTemp >= 21.0 && longTemp < 33.0) {
                zoneNumber = 35;
            } else if (longTemp >= 33.0 && longTemp < 42.0) {
                zoneNumber = 37;
            }
        }

        return UtmZone.from(zoneNumber, UtmZone.getLetter(zoneNumber, geoPoint.getLatitude()));
    }
}