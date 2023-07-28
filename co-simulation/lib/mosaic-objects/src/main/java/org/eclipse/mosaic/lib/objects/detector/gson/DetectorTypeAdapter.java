/*
 * Copyright (C) 2023 LEIDOS.
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
package org.eclipse.mosaic.lib.objects.detector.gson;

import java.lang.reflect.Type;

import org.eclipse.mosaic.lib.objects.detector.DetectorType;

import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonParseException;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

public class DetectorTypeAdapter implements JsonSerializer<DetectorType>, JsonDeserializer<DetectorType> {

    @Override
    public DetectorType deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context)
            throws JsonParseException {
        return DetectorType.fromName(json.getAsString());
    }

    @Override
    public JsonElement serialize(DetectorType src, Type typeOfSrc, JsonSerializationContext context) {
        return context.serialize(src.getLabel());
    }
    
}
