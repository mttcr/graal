/*
 * Copyright (c) 2021, 2021, Oracle and/or its affiliates. All rights reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.  Oracle designates this
 * particular file as subject to the "Classpath" exception as provided
 * by Oracle in the LICENSE file that accompanied this code.
 *
 * This code is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * version 2 for more details (a copy is included in the LICENSE file that
 * accompanied this code).
 *
 * You should have received a copy of the GNU General Public License version
 * 2 along with this work; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Please contact Oracle, 500 Oracle Parkway, Redwood Shores, CA 94065 USA
 * or visit www.oracle.com if you need additional information or have any
 * questions.
 */
package org.graalvm.nativeimage.impl;

import java.util.Objects;

public class ConfigurationPredicate {
    private String typeReachability;
    public static ConfigurationPredicate DEFAULT_CONFIGRATION_PREDICATE = new ConfigurationPredicate("java.lang.Object");

    public static ConfigurationPredicate create(String typeReachability) {
        if (DEFAULT_CONFIGRATION_PREDICATE.typeReachability.equals(typeReachability)) {
            return DEFAULT_CONFIGRATION_PREDICATE;
        }
        return new ConfigurationPredicate(typeReachability);
    }

    private ConfigurationPredicate(String typeReachability) {
        this.typeReachability = typeReachability;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (o == null || getClass() != o.getClass())
            return false;
        ConfigurationPredicate predicate = (ConfigurationPredicate) o;
        return Objects.equals(typeReachability, predicate.typeReachability);
    }

    @Override
    public int hashCode() {
        return Objects.hash(typeReachability);
    }
}
