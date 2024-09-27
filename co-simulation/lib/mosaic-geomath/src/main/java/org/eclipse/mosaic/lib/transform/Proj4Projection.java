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
        this.x_offset = x_offset;
        this.y_offset = y_offset;

        // The proj library being used currently doesn't support use of vertical keywords
        //such as geodgrids and vunits, resulting in no vertical accuracy in the transforms calculated
        //Isssue tracked under: https://github.com/locationtech/proj4j/issues/20
        //Remove unaccepted parameters from georeference
        String regex = "(\\+geoidgrids=[^\\s]+\\s?)|(\\+vunits=[^\\s]+\\s?)";
        georef.replaceAll(regex, "").trim();


        this.cartesianReferenceString = georef;
        this.wgs84ReferenceString = "EPSG:4326";

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
        getGeoCalculator().distanceBetween(geoOrigin, geographic, result);
        return result;
    }

    @Override
    public MutableGeoPoint vectorToGeographic(Vector3d vector3d, MutableGeoPoint result) {
        getGeoCalculator().pointFromDirection(geoOrigin, vector3d, result);
        return result;
    }

    @Override
    public MutableCartesianPoint geographicToCartesian(GeoPoint geographic, MutableCartesianPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(geographic.getLongitude(),geographic.getLatitude());
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(cartesianCRS, wgs84CRS);

        transform.transform(sourceCoord, targetCoord);
        result.set(targetCoord.x, targetCoord.y, 0.0);

        return result;
    }

    @Override
    public MutableGeoPoint cartesianToGeographic(CartesianPoint cartesian, MutableGeoPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(cartesian.getX() - this.x_offset, cartesian.getY() - this.y_offset);
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(cartesianCRS, wgs84CRS);

        transform.transform(sourceCoord, targetCoord);
        //Transform returned as lat,lon = (targetCoord.y, targetCoord.x)
        result.set(targetCoord.y, targetCoord.x, 0.0);

        return result;
    }

    @Override
    public Vector3d utmToVector(UtmPoint utm, Vector3d result) {
        return geographicToVector(utmToGeographic(utm));
    }

    @Override
    public MutableUtmPoint vectorToUtm(Vector3d vector, MutableUtmPoint result) {
        return geographicToUtm(vectorToGeographic(vector), result);
    }

    @Override
    public MutableUtmPoint geographicToUtm(GeoPoint geoPoint, MutableUtmPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(geoPoint.getLongitude(),geoPoint.getLatitude());
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(cartesianCRS, utmCRS);

        transform.transform(sourceCoord, targetCoord);
        result.set(targetCoord.x, targetCoord.y, 0.0, getUTMZone(geoPoint));

        return result;
    }

    @Override
    public MutableGeoPoint utmToGeographic(UtmPoint utmPoint, MutableGeoPoint result) {

        ProjCoordinate sourceCoord = new ProjCoordinate(utmPoint.getEasting(), utmPoint.getNorthing());
        ProjCoordinate targetCoord = new ProjCoordinate();
        CoordinateTransformFactory ctFactory = new CoordinateTransformFactory();
        CoordinateTransform transform = ctFactory.createTransform(utmCRS, wgs84CRS);

        transform.transform(sourceCoord, targetCoord);
        //Transform returned as lon,lat = (targetCoord.x, targetCoord.y)
        result.set(targetCoord.x, targetCoord.y, 0.0);

        return result;
    }

    public UtmZone getUTMZone(GeoPoint geoPoint){
        int zoneNumber;

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