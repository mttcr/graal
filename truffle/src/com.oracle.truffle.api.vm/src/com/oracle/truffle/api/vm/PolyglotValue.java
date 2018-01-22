/*
 * Copyright (c) 2017, Oracle and/or its affiliates. All rights reserved.
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
package com.oracle.truffle.api.vm;

import static com.oracle.truffle.api.vm.PolyglotImpl.wrapGuestException;
import static com.oracle.truffle.api.vm.VMAccessor.LANGUAGE;

import java.util.AbstractSet;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;

import org.graalvm.polyglot.Language;
import org.graalvm.polyglot.PolyglotException;
import org.graalvm.polyglot.TypeLiteral;
import org.graalvm.polyglot.Value;
import org.graalvm.polyglot.impl.AbstractPolyglotImpl.AbstractValueImpl;

import com.oracle.truffle.api.CallTarget;
import com.oracle.truffle.api.CompilerDirectives;
import com.oracle.truffle.api.CompilerDirectives.CompilationFinal;
import com.oracle.truffle.api.Truffle;
import com.oracle.truffle.api.frame.VirtualFrame;
import com.oracle.truffle.api.interop.ArityException;
import com.oracle.truffle.api.interop.ForeignAccess;
import com.oracle.truffle.api.interop.KeyInfo;
import com.oracle.truffle.api.interop.Message;
import com.oracle.truffle.api.interop.TruffleObject;
import com.oracle.truffle.api.interop.UnknownIdentifierException;
import com.oracle.truffle.api.interop.UnsupportedMessageException;
import com.oracle.truffle.api.interop.UnsupportedTypeException;
import com.oracle.truffle.api.interop.java.JavaInterop;
import com.oracle.truffle.api.nodes.Node;
import com.oracle.truffle.api.nodes.RootNode;
import com.oracle.truffle.api.vm.PolyglotLanguageContext.ToGuestValueNode;
import com.oracle.truffle.api.vm.PolyglotLanguageContext.ToGuestValuesNode;
import com.oracle.truffle.api.vm.PolyglotLanguageContext.ToHostValueNode;

abstract class PolyglotValue extends AbstractValueImpl {

    final PolyglotLanguageContext languageContext;
    final PolyglotImpl impl;

    final CallTarget asClassLiteral;
    final CallTarget asTypeLiteral;

    PolyglotValue(PolyglotLanguageContext context) {
        super(context.getEngine().impl);
        this.impl = context.getEngine().impl;
        this.languageContext = context;
        this.asClassLiteral = Truffle.getRuntime().createCallTarget(new AsClassLiteralNode(this));
        this.asTypeLiteral = Truffle.getRuntime().createCallTarget(new AsTypeLiteralNode(this));
    }

    protected abstract Class<?> getReceiverType();

    protected final String formatSuppliedValues(UnsupportedTypeException e) {
        Object[] suppliedValues = e.getSuppliedValues();
        String[] args = new String[suppliedValues.length];
        for (int i = 0; i < suppliedValues.length; i++) {
            Object value = suppliedValues[i];
            String s = null;
            if (value == null) {
                s = "null";
            } else {
                s = LANGUAGE.toStringIfVisible(languageContext.env, value, false);
            }
            args[i] = s;
        }
        return Arrays.toString(args);
    }

    @Override
    public Value getMetaObject(Object receiver) {
        Object prev = languageContext.enter();
        try {
            Object metaObject = LANGUAGE.findMetaObject(languageContext.env, receiver);
            if (metaObject != null) {
                return newValue(metaObject);
            } else {
                return null;
            }
        } catch (Throwable e) {
            throw wrapGuestException(languageContext, e);
        } finally {
            languageContext.leave(prev);
        }
    }

    private Language getLanguage() {
        return languageContext.language.api;
    }

    @SuppressWarnings("unchecked")
    @Override
    public final <T> T as(Object receiver, Class<T> targetType) {
        return (T) VMAccessor.SPI.callProfiled(asClassLiteral, receiver, targetType);
    }

    @SuppressWarnings("unchecked")
    @Override
    public final <T> T as(Object receiver, TypeLiteral<T> targetType) {
        return (T) VMAccessor.SPI.callProfiled(asTypeLiteral, receiver, targetType);
    }

    @Override
    protected RuntimeException unsupported(Object receiver, String message, String useToCheck) {
        Object prev = languageContext.enter();
        try {
            Object metaObject = LANGUAGE.findMetaObject(languageContext.env, receiver);
            String typeName = LANGUAGE.toStringIfVisible(languageContext.env, metaObject, false);
            String languageName = getLanguage().getName();

            throw new PolyglotUnsupportedException(
                            String.format("Unsupported operation %s.%s for receiver type %s and language %s. You can ensure that the operation is supported using %s.%s.",
                                            Value.class.getSimpleName(), message, typeName, languageName, Value.class.getSimpleName(), useToCheck));
        } catch (Throwable e) {
            throw wrapGuestException(languageContext, e);
        } finally {
            languageContext.leave(prev);
        }
    }

    @Override
    protected RuntimeException npe(Object receiver, String message, String useToCheck) {
        Object prev = languageContext.enter();
        try {
            Object metaObject = LANGUAGE.findMetaObject(languageContext.env, receiver);
            String typeName = LANGUAGE.toStringIfVisible(languageContext.env, metaObject, false);
            String languageName = getLanguage().getName();

            throw new PolyglotNullPointerException(
                            String.format("Unsupported operation %s.%s for receiver type %s and language %s. You can ensure that the operation is supported using %s.%s.",
                                            Value.class.getSimpleName(), message, typeName, languageName, Value.class.getSimpleName(), useToCheck));
        } catch (Throwable e) {
            throw wrapGuestException(languageContext, e);
        } finally {
            languageContext.leave(prev);
        }
    }

    @Override
    protected RuntimeException classcast(Object receiver, String castTypeName, String useToCheck) {
        Object prev = languageContext.enter();
        try {
            Object metaObject = LANGUAGE.findMetaObject(languageContext.env, receiver);
            String typeName = LANGUAGE.toStringIfVisible(languageContext.env, metaObject, false);
            String languageName = getLanguage().getName();

            throw new PolyglotClassCastException(
                            String.format("Cannot cast value using %s.%s with receiver type '%s' of language '%s'. You can ensure that the value can be cast using %s.%s.",
                                            Value.class.getSimpleName(), castTypeName, typeName, languageName, Value.class.getSimpleName(), useToCheck));
        } catch (Throwable e) {
            throw wrapGuestException(languageContext, e);
        } finally {
            languageContext.leave(prev);
        }
    }

    @Override
    public String toString(Object receiver) {
        Object prev = languageContext.enter();
        try {
            return LANGUAGE.toStringIfVisible(languageContext.env, receiver, false);
        } catch (Throwable e) {
            throw wrapGuestException(languageContext, e);
        } finally {
            languageContext.leave(prev);
        }
    }

    protected final Value newValue(Object receiver) {
        return languageContext.toHostValue(receiver);
    }

    static PolyglotValue createInteropValueCache(PolyglotLanguageContext languageContext, TruffleObject receiver, Class<?> receiverType) {
        return new Interop(languageContext, receiver, receiverType);
    }

    static void createDefaultValueCaches(PolyglotLanguageContext context) {
        Map<Class<?>, PolyglotValue> valueCache = context.valueCache;
        valueCache.put(Boolean.class, new BooleanValueCache(context));
        valueCache.put(Byte.class, new ByteValueCache(context));
        valueCache.put(Short.class, new ShortValueCache(context));
        valueCache.put(Integer.class, new IntValueCache(context));
        valueCache.put(Long.class, new LongValueCache(context));
        valueCache.put(Float.class, new FloatValueCache(context));
        valueCache.put(Double.class, new DoubleValueCache(context));
        valueCache.put(String.class, new StringValueCache(context));
        valueCache.put(Character.class, new CharacterValueCache(context));
    }

    private static class AsClassLiteralNode extends Interop.PolyglotNode {

        @Child Node toJava = VMAccessor.JAVAINTEROP.createToJavaNode();

        protected AsClassLiteralNode(PolyglotValue interop) {
            super(interop);
        }

        @Override
        protected Class<?>[] getArgumentTypes() {
            return new Class[]{polyglot.getReceiverType(), Class.class};
        }

        @Override
        protected String getOperationName() {
            return "as";
        }

        @Override
        protected Object executeImpl(Object receiver, Object[] args) {
            return VMAccessor.JAVAINTEROP.toJava(toJava, (Class<?>) args[1], null, args[0], polyglot.languageContext);
        }

    }

    private static class AsTypeLiteralNode extends Interop.PolyglotNode {

        @Child Node toJava = VMAccessor.JAVAINTEROP.createToJavaNode();

        protected AsTypeLiteralNode(PolyglotValue interop) {
            super(interop);
        }

        @Override
        protected Class<?>[] getArgumentTypes() {
            return new Class[]{polyglot.getReceiverType(), TypeLiteral.class};
        }

        @Override
        protected String getOperationName() {
            return "as";
        }

        @Override
        protected Object executeImpl(Object receiver, Object[] args) {
            TypeLiteral<?> typeLiteral = (TypeLiteral<?>) args[1];
            return VMAccessor.JAVAINTEROP.toJava(toJava, typeLiteral.getRawType(), typeLiteral.getType(), args[0], polyglot.languageContext);
        }

    }

    private static final class StringValueCache extends PolyglotValue {

        StringValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return String.class;
        }

        @Override
        public boolean isString(Object receiver) {
            return true;
        }

        @Override
        public String asString(Object receiver) {
            return (String) receiver;
        }

    }

    private static final class BooleanValueCache extends PolyglotValue {

        BooleanValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Boolean.class;
        }

        @Override
        public boolean isBoolean(Object receiver) {
            return true;
        }

        @Override
        public boolean asBoolean(Object receiver) {
            return (boolean) receiver;
        }

    }

    private static final class ByteValueCache extends PolyglotValue {

        ByteValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Byte.class;
        }

        @Override
        public boolean isNumber(Object receiver) {
            return true;
        }

        @Override
        public boolean fitsInByte(Object receiver) {
            return true;
        }

        @Override
        public byte asByte(Object receiver) {
            return (byte) receiver;
        }

        @Override
        public boolean fitsInShort(Object receiver) {
            return true;
        }

        @Override
        public short asShort(Object receiver) {
            return (byte) receiver;
        }

        @Override
        public boolean fitsInInt(Object receiver) {
            return true;
        }

        @Override
        public int asInt(Object receiver) {
            return (byte) receiver;
        }

        @Override
        public boolean fitsInLong(Object receiver) {
            return true;
        }

        @Override
        public long asLong(Object receiver) {
            return (byte) receiver;
        }

        @Override
        public boolean fitsInFloat(Object receiver) {
            return true;
        }

        @Override
        public float asFloat(Object receiver) {
            return (byte) receiver;
        }

        @Override
        public boolean fitsInDouble(Object receiver) {
            return true;
        }

        @Override
        public double asDouble(Object receiver) {
            return (byte) receiver;
        }

    }

    private static final class ShortValueCache extends PolyglotValue {

        ShortValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Short.class;
        }

        @Override
        public boolean isNumber(Object receiver) {
            return true;
        }

        @Override
        public boolean fitsInByte(Object receiver) {
            short originalReceiver = (short) receiver;
            byte castValue = (byte) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public byte asByte(Object receiver) {
            short originalReceiver = (short) receiver;
            byte castValue = (byte) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asByte(receiver);
            }
        }

        @Override
        public boolean fitsInShort(Object receiver) {
            return true;
        }

        @Override
        public short asShort(Object receiver) {
            return (short) receiver;
        }

        @Override
        public boolean fitsInInt(Object receiver) {
            return true;
        }

        @Override
        public int asInt(Object receiver) {
            return (short) receiver;
        }

        @Override
        public boolean fitsInLong(Object receiver) {
            return true;
        }

        @Override
        public long asLong(Object receiver) {
            return (short) receiver;
        }

        @Override
        public boolean fitsInFloat(Object receiver) {
            return true;
        }

        @Override
        public float asFloat(Object receiver) {
            return (short) receiver;
        }

        @Override
        public boolean fitsInDouble(Object receiver) {
            return true;
        }

        @Override
        public double asDouble(Object receiver) {
            return (short) receiver;
        }

    }

    private static final class CharacterValueCache extends PolyglotValue {

        CharacterValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Character.class;
        }

        @Override
        public boolean isString(Object receiver) {
            return true;
        }

        @Override
        public String asString(Object receiver) {
            return String.valueOf((char) receiver);
        }
    }

    private static final class LongValueCache extends PolyglotValue {

        LongValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Long.class;
        }

        @Override
        public boolean isNumber(Object receiver) {
            return true;
        }

        @Override
        public boolean fitsInByte(Object receiver) {
            long originalReceiver = (long) receiver;
            byte castValue = (byte) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public byte asByte(Object receiver) {
            long originalReceiver = (long) receiver;
            byte castValue = (byte) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asByte(receiver);
            }
        }

        @Override
        public boolean fitsInInt(Object receiver) {
            long originalReceiver = (long) receiver;
            int castValue = (int) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public int asInt(Object receiver) {
            long originalReceiver = (long) receiver;
            int castValue = (int) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asInt(receiver);
            }
        }

        @Override
        public boolean fitsInLong(Object receiver) {
            return true;
        }

        @Override
        public long asLong(Object receiver) {
            return (long) receiver;
        }

        @Override
        public boolean fitsInFloat(Object receiver) {
            long originalReceiver = (long) receiver;
            float castValue = originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public float asFloat(Object receiver) {
            long originalReceiver = (long) receiver;
            float castValue = originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asFloat(receiver);
            }
        }

        @Override
        public boolean fitsInDouble(Object receiver) {
            long originalReceiver = (long) receiver;
            double castValue = originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public double asDouble(Object receiver) {
            long originalReceiver = (long) receiver;
            double castValue = originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asDouble(receiver);
            }
        }

        @Override
        public boolean fitsInShort(Object receiver) {
            long originalReceiver = (long) receiver;
            short castValue = (short) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public short asShort(Object receiver) {
            long originalReceiver = (long) receiver;
            short castValue = (short) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asShort(receiver);
            }
        }
    }

    private static final class FloatValueCache extends PolyglotValue {

        FloatValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Float.class;
        }

        @Override
        public boolean isNumber(Object receiver) {
            return true;
        }

        @Override
        public boolean fitsInByte(Object receiver) {
            float originalReceiver = (float) receiver;
            byte castValue = (byte) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public byte asByte(Object receiver) {
            float originalReceiver = (float) receiver;
            byte castValue = (byte) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asByte(receiver);
            }
        }

        @Override
        public boolean fitsInInt(Object receiver) {
            float originalReceiver = (float) receiver;
            int castValue = (int) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public int asInt(Object receiver) {
            float originalReceiver = (float) receiver;
            int castValue = (int) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asInt(receiver);
            }
        }

        @Override
        public boolean fitsInLong(Object receiver) {
            float originalReceiver = (float) receiver;
            long castValue = (long) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public long asLong(Object receiver) {
            float originalReceiver = (float) receiver;
            long castValue = (long) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asLong(receiver);
            }
        }

        @Override
        public boolean fitsInFloat(Object receiver) {
            return true;
        }

        @Override
        public float asFloat(Object receiver) {
            return (float) receiver;
        }

        @Override
        public boolean fitsInDouble(Object receiver) {
            return true;
        }

        @Override
        public double asDouble(Object receiver) {
            return (float) receiver;
        }

        @Override
        public boolean fitsInShort(Object receiver) {
            float originalReceiver = (float) receiver;
            short castValue = (short) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public short asShort(Object receiver) {
            float originalReceiver = (float) receiver;
            short castValue = (short) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asShort(receiver);
            }
        }
    }

    private static final class DoubleValueCache extends PolyglotValue {

        DoubleValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Double.class;
        }

        @Override
        public boolean isNumber(Object receiver) {
            return true;
        }

        @Override
        public boolean fitsInByte(Object receiver) {
            double originalReceiver = (double) receiver;
            byte castValue = (byte) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public byte asByte(Object receiver) {
            double originalReceiver = (double) receiver;
            byte castValue = (byte) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asByte(receiver);
            }
        }

        @Override
        public boolean fitsInInt(Object receiver) {
            double originalReceiver = (double) receiver;
            int castValue = (int) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public int asInt(Object receiver) {
            double originalReceiver = (double) receiver;
            int castValue = (int) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asInt(receiver);
            }
        }

        @Override
        public boolean fitsInLong(Object receiver) {
            double originalReceiver = (double) receiver;
            long castValue = (long) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public long asLong(Object receiver) {
            double originalReceiver = (double) receiver;
            long castValue = (long) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asLong(receiver);
            }
        }

        @Override
        public boolean fitsInFloat(Object receiver) {
            double originalReceiver = (double) receiver;
            float castValue = (float) originalReceiver;
            return castValue == originalReceiver ||
                            (Double.isNaN(originalReceiver) && Float.isNaN(castValue));
        }

        @Override
        public float asFloat(Object receiver) {
            double originalReceiver = (double) receiver;
            float castValue = (float) originalReceiver;
            if (originalReceiver == castValue ||
                            (Double.isNaN(originalReceiver) && Float.isNaN(castValue))) {
                return castValue;
            } else {
                return super.asFloat(receiver);
            }
        }

        @Override
        public boolean fitsInDouble(Object receiver) {
            return true;
        }

        @Override
        public double asDouble(Object receiver) {
            return (double) receiver;
        }

        @Override
        public boolean fitsInShort(Object receiver) {
            double originalReceiver = (double) receiver;
            short castValue = (short) originalReceiver;
            return originalReceiver == castValue;
        }

        @Override
        public short asShort(Object receiver) {
            double originalReceiver = (double) receiver;
            short castValue = (short) originalReceiver;
            if (originalReceiver == castValue) {
                return castValue;
            } else {
                return super.asShort(receiver);
            }
        }
    }

    private static final class IntValueCache extends PolyglotValue {

        IntValueCache(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Integer.class;
        }

        @Override
        public boolean isNumber(Object receiver) {
            return true;
        }

        @Override
        public boolean fitsInInt(Object receiver) {
            return true;
        }

        @Override
        public int asInt(Object receiver) {
            return (int) receiver;
        }

        @Override
        public boolean fitsInLong(Object receiver) {
            return true;
        }

        @Override
        public long asLong(Object receiver) {
            return (int) receiver;
        }

        @Override
        public boolean fitsInDouble(Object receiver) {
            return true;
        }

        @Override
        public double asDouble(Object receiver) {
            return (int) receiver;
        }

        @Override
        public boolean fitsInByte(Object receiver) {
            int intReceiver = (int) receiver;
            byte castValue = (byte) intReceiver;
            return intReceiver == castValue;
        }

        @Override
        public byte asByte(Object receiver) {
            int intReceiver = (int) receiver;
            byte castValue = (byte) intReceiver;
            if (intReceiver == castValue) {
                return castValue;
            } else {
                return super.asByte(receiver);
            }
        }

        @Override
        public boolean fitsInFloat(Object receiver) {
            int intReceiver = (int) receiver;
            float castValue = intReceiver;
            return intReceiver == (int) castValue;
        }

        @Override
        public float asFloat(Object receiver) {
            int intReceiver = (int) receiver;
            float castValue = intReceiver;
            if (intReceiver == (int) castValue) {
                return castValue;
            } else {
                return super.asFloat(receiver);
            }
        }

        @Override
        public boolean fitsInShort(Object receiver) {
            int intReceiver = (int) receiver;
            short castValue = (short) intReceiver;
            return intReceiver == castValue;
        }

        @Override
        public short asShort(Object receiver) {
            int intReceiver = (int) receiver;
            short castValue = (short) intReceiver;
            if (intReceiver == castValue) {
                return castValue;
            } else {
                return super.asShort(receiver);
            }
        }
    }

    static final class Default extends PolyglotValue {

        Default(PolyglotLanguageContext context) {
            super(context);
        }

        @Override
        protected Class<?> getReceiverType() {
            return Object.class;
        }

    }

    private static final class Interop extends PolyglotValue {

        final Node keysNode = Message.KEYS.createNode();
        final Node keyInfoNode = Message.KEY_INFO.createNode();
        final Node keysSizeNode = Message.GET_SIZE.createNode();
        final Node keysReadNode = Message.READ.createNode();

        final CallTarget isNativePointer;
        final CallTarget asNativePointer;
        final CallTarget hasArrayElements;
        final CallTarget getArrayElement;
        final CallTarget setArrayElement;
        final CallTarget getArraySize;
        final CallTarget hasMembers;
        final CallTarget hasMember;
        final CallTarget getMember;
        final CallTarget putMember;
        final CallTarget isNull;
        final CallTarget canExecute;
        final CallTarget execute;
        final CallTarget canInstantiate;
        final CallTarget newInstance;
        final CallTarget executeNoArgs;
        final CallTarget executeVoid;
        final CallTarget executeVoidNoArgs;
        final CallTarget asPrimitive;

        final Class<?> receiverType;
        final boolean isProxy;
        final boolean isJava;

        Interop(PolyglotLanguageContext context, TruffleObject receiver, Class<?> receiverType) {
            super(context);
            this.receiverType = receiverType;
            this.isNativePointer = createTarget(new IsNativePointerNode(this));
            this.asNativePointer = createTarget(new AsNativePointerNode(this));
            this.hasArrayElements = createTarget(new HasArrayElementsNode(this));
            this.getArrayElement = createTarget(new GetArrayElementNode(this));
            this.setArrayElement = createTarget(new SetArrayElementNode(this));
            this.getArraySize = createTarget(new GetArraySizeNode(this));
            this.hasMember = createTarget(new HasMemberNode(this));
            this.getMember = createTarget(new GetMemberNode(this));
            this.putMember = createTarget(new PutMemberNode(this));
            this.isNull = createTarget(new IsNullNode(this));
            this.execute = createTarget(new ExecuteNode(this));
            this.executeNoArgs = createTarget(new ExecuteNoArgsNode(this));
            this.executeVoid = createTarget(new ExecuteVoidNode(this));
            this.executeVoidNoArgs = createTarget(new ExecuteVoidNoArgsNode(this));
            this.newInstance = createTarget(new NewInstanceNode(this));
            this.canInstantiate = createTarget(new CanInstantiateNode(this));
            this.canExecute = createTarget(new CanExecuteNode(this));
            this.hasMembers = createTarget(new HasMembersNode(this));
            this.asPrimitive = createTarget(new AsPrimitiveNode(this));
            this.isProxy = PolyglotProxy.isProxyGuestObject(receiver);
            this.isJava = JavaInterop.isJavaObject(receiver);
        }

        private static CallTarget createTarget(PolyglotNode root) {
            CallTarget target = Truffle.getRuntime().createCallTarget(root);
            Class<?>[] types = root.getArgumentTypes();
            if (types != null) {
                VMAccessor.SPI.initializeProfile(target, types);
            }
            return target;
        }

        @Override
        protected final Class<?> getReceiverType() {
            return receiverType;
        }

        @Override
        public boolean isNativePointer(Object receiver) {
            return (boolean) VMAccessor.SPI.callProfiled(isNativePointer, receiver);
        }

        @Override
        public boolean hasArrayElements(Object receiver) {
            return (boolean) VMAccessor.SPI.callProfiled(hasArrayElements, receiver);
        }

        @Override
        public Value getArrayElement(Object receiver, long index) {
            return (Value) VMAccessor.SPI.callProfiled(getArrayElement, receiver, index);
        }

        @Override
        public void setArrayElement(Object receiver, long index, Object value) {
            VMAccessor.SPI.callProfiled(setArrayElement, receiver, index, value);
        }

        @Override
        public long getArraySize(Object receiver) {
            return (long) VMAccessor.SPI.callProfiled(getArraySize, receiver);
        }

        @Override
        public boolean hasMembers(Object receiver) {
            return (boolean) hasMembers.call(receiver);
        }

        @Override
        public Value getMember(Object receiver, String key) {
            return (Value) VMAccessor.SPI.callProfiled(getMember, receiver, key);
        }

        @Override
        public boolean hasMember(Object receiver, String key) {
            return (boolean) VMAccessor.SPI.callProfiled(hasMember, receiver, key);
        }

        @Override
        public void putMember(Object receiver, String key, Object member) {
            VMAccessor.SPI.callProfiled(putMember, receiver, key, member);
        }

        @Override
        public Set<String> getMemberKeys(Object receiver) {
            Object prev = languageContext.enter();
            try {
                try {
                    final Object keys = ForeignAccess.sendKeys(keysNode, (TruffleObject) receiver, false);
                    if (!(keys instanceof TruffleObject)) {
                        return Collections.emptySet();
                    }
                    return new MemberSet((TruffleObject) receiver, (TruffleObject) keys);
                } catch (UnsupportedMessageException e) {
                    return Collections.emptySet();
                }
            } catch (Throwable e) {
                throw wrapGuestException(languageContext, e);
            } finally {
                languageContext.leave(prev);
            }
        }

        @Override
        public long asNativePointer(Object receiver) {
            return (long) VMAccessor.SPI.callProfiled(asNativePointer, receiver);
        }

        @Override
        public boolean isHostObject(Object receiver) {
            return isJava;
        }

        @Override
        public boolean isProxyObject(Object receiver) {
            return isProxy;
        }

        @Override
        public Object asProxyObject(Object receiver) {
            if (isProxy) {
                return PolyglotProxy.toProxyHostObject((TruffleObject) receiver);
            } else {
                return super.asProxyObject(receiver);
            }
        }

        @Override
        public Object asHostObject(Object receiver) {
            TruffleObject castReceiver = (TruffleObject) receiver;
            if (isJava) {
                return JavaInterop.asJavaObject(castReceiver);
            } else {
                return super.asHostObject(receiver);
            }
        }

        @Override
        public boolean isNull(Object receiver) {
            return (boolean) VMAccessor.SPI.callProfiled(isNull, receiver);
        }

        @Override
        public boolean canExecute(Object receiver) {
            return (boolean) VMAccessor.SPI.callProfiled(canExecute, receiver);
        }

        @Override
        public void executeVoid(Object receiver, Object[] arguments) {
            VMAccessor.SPI.callProfiled(executeVoid, receiver, arguments);
        }

        @Override
        public void executeVoid(Object receiver) {
            VMAccessor.SPI.callProfiled(executeVoidNoArgs, receiver);
        }

        @Override
        public Value execute(Object receiver, Object[] arguments) {
            return (Value) VMAccessor.SPI.callProfiled(execute, receiver, arguments);
        }

        @Override
        public Value execute(Object receiver) {
            return (Value) VMAccessor.SPI.callProfiled(executeNoArgs, receiver);
        }

        @Override
        public boolean canInstantiate(Object receiver) {
            return (boolean) canInstantiate.call(receiver);
        }

        @Override
        public Value newInstance(Object receiver, Object[] arguments) {
            return (Value) newInstance.call(receiver, arguments);
        }

        private static PolyglotException error(String message, Exception cause) {
            throw new PolyglotUnsupportedException(message, cause);
        }

        private Object asPrimitive(Object receiver) {
            return VMAccessor.SPI.callProfiled(asPrimitive, receiver);
        }

        private PolyglotValue getPrimitiveCache(Object primitive) {
            assert primitive != null;
            PolyglotValue cache = languageContext.valueCache.get(primitive.getClass());
            if (cache == null) {
                // TODO maybe this should be an assertion here because it likely means
                // that unbox returned an invalid value.
                cache = languageContext.defaultValueCache;
            }
            return cache;
        }

        @Override
        public boolean isNumber(Object receiver) {
            return asPrimitive(receiver) instanceof Number;
        }

        @Override
        public boolean fitsInByte(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.fitsInByte(receiver);
            }
            return getPrimitiveCache(primitive).fitsInByte(primitive);
        }

        @Override
        public byte asByte(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asByte(receiver);
            }
            return getPrimitiveCache(primitive).asByte(primitive);
        }

        @Override
        public boolean isString(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.isString(receiver);
            }
            return getPrimitiveCache(primitive).isString(primitive);
        }

        @Override
        public String asString(Object receiver) {
            if (isNull(receiver)) {
                return null;
            }
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asString(receiver);
            }
            return getPrimitiveCache(primitive).asString(primitive);
        }

        @Override
        public boolean fitsInInt(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.fitsInInt(receiver);
            }
            return getPrimitiveCache(primitive).fitsInInt(primitive);
        }

        @Override
        public int asInt(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asInt(receiver);
            }
            return getPrimitiveCache(primitive).asInt(primitive);
        }

        @Override
        public boolean isBoolean(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.isBoolean(receiver);
            }
            return getPrimitiveCache(primitive).isBoolean(primitive);
        }

        @Override
        public boolean asBoolean(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asBoolean(receiver);
            }
            return getPrimitiveCache(primitive).asBoolean(primitive);
        }

        @Override
        public boolean fitsInFloat(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.fitsInFloat(receiver);
            }
            return getPrimitiveCache(primitive).fitsInFloat(primitive);
        }

        @Override
        public float asFloat(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asFloat(receiver);
            }
            return getPrimitiveCache(primitive).asFloat(primitive);
        }

        @Override
        public boolean fitsInDouble(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.fitsInDouble(receiver);
            }
            return getPrimitiveCache(primitive).fitsInDouble(primitive);
        }

        @Override
        public double asDouble(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asDouble(receiver);
            }
            return getPrimitiveCache(primitive).asDouble(primitive);
        }

        @Override
        public boolean fitsInLong(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.fitsInLong(receiver);
            }
            return getPrimitiveCache(primitive).fitsInLong(primitive);
        }

        @Override
        public long asLong(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asLong(receiver);
            }
            return getPrimitiveCache(primitive).asLong(primitive);
        }

        @Override
        public boolean fitsInShort(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.fitsInShort(receiver);
            }
            return getPrimitiveCache(primitive).fitsInShort(primitive);
        }

        @Override
        public short asShort(Object receiver) {
            Object primitive = asPrimitive(receiver);
            if (primitive == null) {
                return super.asShort(receiver);
            }
            return getPrimitiveCache(primitive).asShort(primitive);
        }

        private abstract static class PolyglotNode extends RootNode {

            protected final PolyglotValue polyglot;

            protected abstract String getOperationName();

            @CompilationFinal private boolean seenEnter;
            @CompilationFinal private boolean seenNonEnter;

            protected PolyglotNode(PolyglotValue polyglot) {
                super(null);
                this.polyglot = polyglot;
            }

            protected abstract Class<?>[] getArgumentTypes();

            @Override
            public final Object execute(VirtualFrame frame) {
                Object[] args = frame.getArguments();
                Object receiver = polyglot.getReceiverType().cast(args[0]);
                PolyglotContextImpl context = polyglot.languageContext.context;
                boolean needsEnter = context.needsEnter();
                Object prev;
                if (needsEnter) {
                    if (!seenEnter) {
                        CompilerDirectives.transferToInterpreterAndInvalidate();
                        seenEnter = true;
                    }
                    prev = context.enter();
                } else {
                    if (!seenNonEnter) {
                        CompilerDirectives.transferToInterpreterAndInvalidate();
                        seenNonEnter = true;
                    }
                    prev = null;
                }
                try {
                    return executeImpl(receiver, args);
                } catch (Throwable e) {
                    CompilerDirectives.transferToInterpreter();
                    throw wrapGuestException(polyglot.languageContext, e);
                } finally {
                    if (needsEnter) {
                        context.leave(prev);
                    }
                }
            }

            protected abstract Object executeImpl(Object receiver, Object[] args);

            @Override
            public final String getName() {
                return "org.graalvm.polyglot.Value<" + polyglot.getReceiverType().getSimpleName() + ">." + getOperationName();
            }

            @Override
            public final String toString() {
                return getName();
            }

        }

        private static class IsNativePointerNode extends PolyglotNode {

            @Child private Node isPointerNode = Message.IS_POINTER.createNode();

            protected IsNativePointerNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected String getOperationName() {
                return "isNativePointer";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return ForeignAccess.sendIsPointer(isPointerNode, (TruffleObject) receiver);
            }

        }

        private static class AsNativePointerNode extends PolyglotNode {

            @Child private Node asPointerNode = Message.AS_POINTER.createNode();

            protected AsNativePointerNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected String getOperationName() {
                return "asNativePointer";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                try {
                    return ForeignAccess.sendAsPointer(asPointerNode, (TruffleObject) receiver);
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    return polyglot.asNativePointerUnsupported(receiver);
                }
            }

        }

        private static class HasArrayElementsNode extends PolyglotNode {

            @Child private Node hasSizeNode = Message.HAS_SIZE.createNode();

            protected HasArrayElementsNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected String getOperationName() {
                return "hasArrayElements";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return ForeignAccess.sendHasSize(hasSizeNode, (TruffleObject) receiver);
            }

        }

        private static class GetArrayElementNode extends PolyglotNode {

            @Child private Node readArrayNode = Message.READ.createNode();
            private final ToHostValueNode toHostValue = polyglot.languageContext.createToHostValue();

            protected GetArrayElementNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), Long.class};
            }

            @Override
            protected String getOperationName() {
                return "getArrayElement";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                long index = (long) args[1];
                try {
                    return toHostValue.execute(ForeignAccess.sendRead(readArrayNode, (TruffleObject) receiver, index));
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    return polyglot.getArrayElementUnsupported(receiver);
                } catch (UnknownIdentifierException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw error(String.format("Invalid provided index %s for object %s.", index, toString()), e);
                }
            }

        }

        private static class SetArrayElementNode extends PolyglotNode {

            @Child private Node writeArrayNode = Message.WRITE.createNode();

            private final ToGuestValueNode toGuestValue = ToGuestValueNode.create();

            protected SetArrayElementNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), Long.class, Object.class};
            }

            @Override
            protected String getOperationName() {
                return "setArrayElement";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                long index = (long) args[1];
                Object value = args[2];
                try {
                    ForeignAccess.sendWrite(writeArrayNode, (TruffleObject) receiver, index, toGuestValue.apply(polyglot.languageContext, value));
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    polyglot.setArrayElementUnsupported(receiver);
                } catch (UnknownIdentifierException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw error(String.format("Invalid provided index %s for object %s.", index, toString()), e);
                } catch (UnsupportedTypeException e) {
                    CompilerDirectives.transferToInterpreter();
                    String arguments = polyglot.formatSuppliedValues(e);
                    throw error(String.format("Invalid array value provided %s when writing to %s at index %s.", arguments, toString(), index), e);
                }
                return null;
            }
        }

        private static class GetArraySizeNode extends PolyglotNode {

            @Child private Node getSizeNode = Message.GET_SIZE.createNode();

            protected GetArraySizeNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected String getOperationName() {
                return "getArraySize";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                try {
                    return ((Number) ForeignAccess.sendGetSize(getSizeNode, (TruffleObject) receiver)).longValue();
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    return polyglot.getArraySizeUnsupported(receiver);
                }
            }

        }

        private static class GetMemberNode extends PolyglotNode {

            @Child private Node readMemberNode = Message.READ.createNode();

            private final ToHostValueNode toHostValue = polyglot.languageContext.createToHostValue();

            protected GetMemberNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), String.class};
            }

            @Override
            protected String getOperationName() {
                return "getMember";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                String key = (String) args[1];
                try {
                    return toHostValue.execute(ForeignAccess.sendRead(readMemberNode, (TruffleObject) receiver, key));
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    return polyglot.getMemberUnsupported(receiver, key);
                } catch (UnknownIdentifierException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw error(String.format("Unknown provided key %s for object %s.", key, toString()), e);
                }
            }

        }

        private static class PutMemberNode extends PolyglotNode {

            @Child private Node writeMemberNode = Message.WRITE.createNode();
            private final ToGuestValueNode toGuestValue = ToGuestValueNode.create();

            protected PutMemberNode(Interop interop) {
                super(interop);
            }

            @Override
            protected String getOperationName() {
                return "putMember";
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), String.class, Object.class};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                String key = (String) args[1];
                Object member = args[2];
                try {
                    ForeignAccess.sendWrite(writeMemberNode, (TruffleObject) receiver, key, toGuestValue.apply(polyglot.languageContext, member));
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    polyglot.putMemberUnsupported(receiver);
                } catch (UnknownIdentifierException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw error(String.format("Unknown provided key  %s for object %s.", key, toString()), e);
                } catch (UnsupportedTypeException e) {
                    CompilerDirectives.transferToInterpreter();
                    String arguments = polyglot.formatSuppliedValues(e);
                    throw error(String.format("Invalid value provided %s when writing to %s with member key %s.", arguments, toString(), key), e);
                }
                return null;
            }

        }

        private static class IsNullNode extends PolyglotNode {

            @Child private Node isNullNode = Message.IS_NULL.createNode();

            protected IsNullNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected String getOperationName() {
                return "isNull";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return ForeignAccess.sendIsNull(isNullNode, (TruffleObject) receiver);
            }

        }

        private static class HasMembersNode extends PolyglotNode {

            @Child private Node hasKeysNode = Message.HAS_KEYS.createNode();

            protected HasMembersNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected String getOperationName() {
                return "hasMembers";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return ForeignAccess.sendHasKeys(hasKeysNode, (TruffleObject) receiver);
            }

        }

        private static class HasMemberNode extends PolyglotNode {

            final Node keyInfoNode = Message.KEY_INFO.createNode();

            protected HasMemberNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), String.class};
            }

            @Override
            protected String getOperationName() {
                return "hasMember";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                String key = (String) args[1];
                int keyInfo = ForeignAccess.sendKeyInfo(keyInfoNode, (TruffleObject) receiver, key);
                return KeyInfo.isExisting(keyInfo);
            }

        }

        private static class CanExecuteNode extends PolyglotNode {

            @Child private Node isExecutableNode = Message.IS_EXECUTABLE.createNode();

            protected CanExecuteNode(Interop interop) {
                super(interop);
            }

            @Override
            protected String getOperationName() {
                return "canExecute";
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return ForeignAccess.sendIsExecutable(isExecutableNode, (TruffleObject) receiver);
            }

        }

        private static class CanInstantiateNode extends PolyglotNode {

            @Child private Node isInstantiableNode = Message.IS_INSTANTIABLE.createNode();

            protected CanInstantiateNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected String getOperationName() {
                return "canInstantiate";
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return ForeignAccess.sendIsInstantiable(isInstantiableNode, (TruffleObject) receiver);
            }

        }

        private static class AsPrimitiveNode extends PolyglotNode {

            @Child private Node isBoxedNode = Message.IS_BOXED.createNode();
            @Child private Node unboxNode = Message.UNBOX.createNode();

            protected AsPrimitiveNode(Interop interop) {
                super(interop);
            }

            @Override
            protected String getOperationName() {
                return "asPrimitive";
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                if (ForeignAccess.sendIsBoxed(isBoxedNode, (TruffleObject) receiver)) {
                    try {
                        return ForeignAccess.sendUnbox(unboxNode, (TruffleObject) receiver);
                    } catch (UnsupportedMessageException e) {
                        CompilerDirectives.transferToInterpreter();
                        throw new AssertionError("isBoxed returned true but unbox threw unsupported error.");
                    }
                } else {
                    return null;
                }
            }
        }

        private abstract static class AbstractExecuteNode extends PolyglotNode {

            @Child private Node executeNode = Message.createExecute(0).createNode();
            private final ToGuestValuesNode toGuestValues = ToGuestValuesNode.create();

            protected AbstractExecuteNode(Interop interop) {
                super(interop);
            }

            protected final Object executeShared(Object receiver, Object[] args) {
                try {
                    return ForeignAccess.sendExecute(executeNode, (TruffleObject) receiver, toGuestValues.apply(polyglot.languageContext, args));
                } catch (UnsupportedTypeException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw handleUnsupportedType(e);
                } catch (ArityException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw handleInvalidArity(e);
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    return polyglot.executeUnsupported(receiver);
                }
            }

            private PolyglotException handleInvalidArity(ArityException e) {
                int actual = e.getActualArity();
                int expected = e.getExpectedArity();
                return error(String.format("Expected %s number of arguments but got %s when executing %s.", expected, actual, toString()), e);
            }

            private PolyglotException handleUnsupportedType(UnsupportedTypeException e) {
                String arguments = polyglot.formatSuppliedValues(e);
                return error(String.format("Invalid arguments provided %s when executing %s.", arguments, toString()), e);
            }

        }

        private static class ExecuteVoidNode extends AbstractExecuteNode {

            protected ExecuteVoidNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), Object[].class};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                executeShared(receiver, (Object[]) args[1]);
                return null;
            }

            @Override
            protected String getOperationName() {
                return "executeVoid";
            }

        }

        private static class ExecuteVoidNoArgsNode extends AbstractExecuteNode {

            private static final Object[] NO_ARGS = new Object[0];

            protected ExecuteVoidNoArgsNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                executeShared(receiver, NO_ARGS);
                return null;
            }

            @Override
            protected String getOperationName() {
                return "executeVoid";
            }

        }

        private static class ExecuteNode extends AbstractExecuteNode {

            private final ToHostValueNode toHostValue = polyglot.languageContext.createToHostValue();

            protected ExecuteNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), Object[].class};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return toHostValue.execute(executeShared(receiver, (Object[]) args[1]));
            }

            @Override
            protected String getOperationName() {
                return "execute";
            }

        }

        private static class ExecuteNoArgsNode extends AbstractExecuteNode {

            private final ToHostValueNode toHostValue = polyglot.languageContext.createToHostValue();

            protected ExecuteNoArgsNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType()};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                return toHostValue.execute(executeShared(receiver, ExecuteVoidNoArgsNode.NO_ARGS));
            }

            @Override
            protected String getOperationName() {
                return "execute";
            }

        }

        private static class NewInstanceNode extends PolyglotNode {

            @Child private Node newInstanceNode = Message.createNew(0).createNode();
            private final ToGuestValuesNode toGuestValues = ToGuestValuesNode.create();
            private final ToHostValueNode toHostValue = polyglot.languageContext.createToHostValue();

            protected NewInstanceNode(Interop interop) {
                super(interop);
            }

            @Override
            protected Class<?>[] getArgumentTypes() {
                return new Class<?>[]{polyglot.getReceiverType(), Object[].class};
            }

            @Override
            protected Object executeImpl(Object receiver, Object[] args) {
                try {
                    Object[] newInstanceArgs = (Object[]) args[1];
                    return toHostValue.execute(ForeignAccess.sendNew(newInstanceNode, (TruffleObject) receiver, toGuestValues.apply(polyglot.languageContext, newInstanceArgs)));
                } catch (UnsupportedTypeException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw handleUnsupportedType(e);
                } catch (ArityException e) {
                    CompilerDirectives.transferToInterpreter();
                    throw handleInvalidArity(e);
                } catch (UnsupportedMessageException e) {
                    CompilerDirectives.transferToInterpreter();
                    return polyglot.newInstanceUnsupported(receiver);
                }
            }

            private PolyglotException handleInvalidArity(ArityException e) {
                int actual = e.getActualArity();
                int expected = e.getExpectedArity();
                return error(String.format("Expected %s number of arguments but got %s when creating a new instance of %s.", expected, actual, toString()), e);
            }

            private PolyglotException handleUnsupportedType(UnsupportedTypeException e) {
                String arguments = polyglot.formatSuppliedValues(e);
                return error(String.format("Invalid arguments provided %s when creating a new instance of %s.", arguments, toString()), e);
            }

            @Override
            protected String getOperationName() {
                return "newInstance";
            }

        }

        private final class MemberSet extends AbstractSet<String> {

            private final TruffleObject receiver;
            private final TruffleObject keys;
            private int cachedSize = -1;

            MemberSet(TruffleObject receiver, TruffleObject keys) {
                this.receiver = receiver;
                this.keys = keys;
            }

            @Override
            public boolean contains(Object o) {
                if (!(o instanceof String)) {
                    return false;
                }
                Object prev = languageContext.enter();
                try {
                    int keyInfo = ForeignAccess.sendKeyInfo(keyInfoNode, receiver, o);
                    return KeyInfo.isExisting(keyInfo);
                } catch (Throwable e) {
                    throw wrapGuestException(languageContext, e);
                } finally {
                    languageContext.leave(prev);
                }
            }

            @Override
            public Iterator<String> iterator() {
                return new Iterator<String>() {

                    int index = 0;

                    public boolean hasNext() {
                        return index < size();
                    }

                    public String next() {
                        if (index >= size()) {
                            throw new NoSuchElementException();
                        }
                        Object prev = languageContext.enter();
                        try {
                            try {
                                Object result = ForeignAccess.sendRead(keysReadNode, keys, index);
                                index++;
                                return (String) result;
                            } catch (UnsupportedMessageException | UnknownIdentifierException e) {
                                throw new AssertionError("Implementation error: Language must support read messages for keys objects.");
                            }
                        } catch (Throwable e) {
                            throw wrapGuestException(languageContext, e);
                        } finally {
                            languageContext.leave(prev);
                        }
                    }
                };
            }

            @Override
            public int size() {
                if (cachedSize != -1) {
                    return cachedSize;
                }
                Object prev = languageContext.enter();
                try {
                    try {
                        cachedSize = ((Number) ForeignAccess.sendGetSize(keysSizeNode, keys)).intValue();
                    } catch (UnsupportedMessageException e) {
                        return 0;
                    }
                    return cachedSize;
                } catch (Throwable e) {
                    throw wrapGuestException(languageContext, e);
                } finally {
                    languageContext.leave(prev);
                }
            }

        }

    }

}
