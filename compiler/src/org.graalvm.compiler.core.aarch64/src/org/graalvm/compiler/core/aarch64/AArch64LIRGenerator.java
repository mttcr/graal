/*
 * Copyright (c) 2013, 2021, Oracle and/or its affiliates. All rights reserved.
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
package org.graalvm.compiler.core.aarch64;

import static jdk.vm.ci.aarch64.AArch64.sp;
import static org.graalvm.compiler.lir.LIRValueUtil.asJavaConstant;
import static org.graalvm.compiler.lir.LIRValueUtil.asVariable;
import static org.graalvm.compiler.lir.LIRValueUtil.isIntConstant;
import static org.graalvm.compiler.lir.LIRValueUtil.isJavaConstant;

import java.util.function.Function;

import org.graalvm.compiler.asm.aarch64.AArch64Assembler.ConditionFlag;
import org.graalvm.compiler.asm.aarch64.AArch64MacroAssembler;
import org.graalvm.compiler.core.common.LIRKind;
import org.graalvm.compiler.core.common.calc.Condition;
import org.graalvm.compiler.core.common.memory.MemoryOrderMode;
import org.graalvm.compiler.core.common.spi.LIRKindTool;
import org.graalvm.compiler.debug.GraalError;
import org.graalvm.compiler.lir.LIRFrameState;
import org.graalvm.compiler.lir.LIRInstruction;
import org.graalvm.compiler.lir.LIRValueUtil;
import org.graalvm.compiler.lir.LabelRef;
import org.graalvm.compiler.lir.StandardOp;
import org.graalvm.compiler.lir.SwitchStrategy;
import org.graalvm.compiler.lir.Variable;
import org.graalvm.compiler.lir.aarch64.AArch64AddressValue;
import org.graalvm.compiler.lir.aarch64.AArch64ArithmeticOp;
import org.graalvm.compiler.lir.aarch64.AArch64ArrayCompareToOp;
import org.graalvm.compiler.lir.aarch64.AArch64ArrayEqualsOp;
import org.graalvm.compiler.lir.aarch64.AArch64ArrayIndexOfOp;
import org.graalvm.compiler.lir.aarch64.AArch64AtomicMove.AtomicReadAndAddLSEOp;
import org.graalvm.compiler.lir.aarch64.AArch64AtomicMove.AtomicReadAndAddOp;
import org.graalvm.compiler.lir.aarch64.AArch64AtomicMove.AtomicReadAndWriteOp;
import org.graalvm.compiler.lir.aarch64.AArch64AtomicMove.CompareAndSwapOp;
import org.graalvm.compiler.lir.aarch64.AArch64ByteSwap;
import org.graalvm.compiler.lir.aarch64.AArch64CacheWritebackOp;
import org.graalvm.compiler.lir.aarch64.AArch64CacheWritebackPostSyncOp;
import org.graalvm.compiler.lir.aarch64.AArch64Compare;
import org.graalvm.compiler.lir.aarch64.AArch64ControlFlow;
import org.graalvm.compiler.lir.aarch64.AArch64ControlFlow.BranchOp;
import org.graalvm.compiler.lir.aarch64.AArch64ControlFlow.CompareBranchZeroOp;
import org.graalvm.compiler.lir.aarch64.AArch64ControlFlow.CondMoveOp;
import org.graalvm.compiler.lir.aarch64.AArch64ControlFlow.CondSetOp;
import org.graalvm.compiler.lir.aarch64.AArch64ControlFlow.StrategySwitchOp;
import org.graalvm.compiler.lir.aarch64.AArch64ControlFlow.TableSwitchOp;
import org.graalvm.compiler.lir.aarch64.AArch64LIRFlags;
import org.graalvm.compiler.lir.aarch64.AArch64Move;
import org.graalvm.compiler.lir.aarch64.AArch64Move.MembarOp;
import org.graalvm.compiler.lir.aarch64.AArch64PauseOp;
import org.graalvm.compiler.lir.aarch64.AArch64SpeculativeBarrier;
import org.graalvm.compiler.lir.aarch64.AArch64ZapRegistersOp;
import org.graalvm.compiler.lir.aarch64.AArch64ZapStackOp;
import org.graalvm.compiler.lir.aarch64.AArch64ZeroMemoryOp;
import org.graalvm.compiler.lir.gen.LIRGenerationResult;
import org.graalvm.compiler.lir.gen.LIRGenerator;
import org.graalvm.compiler.phases.util.Providers;

import jdk.vm.ci.aarch64.AArch64;
import jdk.vm.ci.aarch64.AArch64Kind;
import jdk.vm.ci.code.CallingConvention;
import jdk.vm.ci.code.Register;
import jdk.vm.ci.code.RegisterValue;
import jdk.vm.ci.code.StackSlot;
import jdk.vm.ci.code.ValueUtil;
import jdk.vm.ci.meta.AllocatableValue;
import jdk.vm.ci.meta.JavaConstant;
import jdk.vm.ci.meta.JavaKind;
import jdk.vm.ci.meta.PlatformKind;
import jdk.vm.ci.meta.PrimitiveConstant;
import jdk.vm.ci.meta.Value;
import jdk.vm.ci.meta.ValueKind;

public abstract class AArch64LIRGenerator extends LIRGenerator {

    public AArch64LIRGenerator(LIRKindTool lirKindTool, AArch64ArithmeticLIRGenerator arithmeticLIRGen, MoveFactory moveFactory, Providers providers, LIRGenerationResult lirGenRes) {
        super(lirKindTool, arithmeticLIRGen, moveFactory, providers, lirGenRes);
    }

    /**
     * Checks whether the supplied constant can be used without loading it into a register for store
     * operations, i.e., on the right hand side of a memory access.
     *
     * @param c The constant to check.
     * @return True if the constant can be used directly, false if the constant needs to be in a
     *         register.
     */
    protected static final boolean canStoreConstant(JavaConstant c) {
        // Our own code never calls this since we can't make a definite statement about whether or
        // not we can inline a constant without knowing what kind of operation we execute. Let's be
        // optimistic here and fix up mistakes later.
        return true;
    }

    /**
     * If val denotes the stackpointer, move it to another location. This is necessary since most
     * ops cannot handle the stackpointer as input or output.
     */
    public AllocatableValue moveSp(AllocatableValue val) {
        if (val instanceof RegisterValue && ((RegisterValue) val).getRegister().equals(sp)) {
            assert val.getPlatformKind() == AArch64Kind.QWORD : "Stackpointer must be long";
            return emitMove(val);
        }
        return val;
    }

    /**
     * AArch64 cannot use anything smaller than a word in any instruction other than load and store.
     */
    @Override
    public <K extends ValueKind<K>> K toRegisterKind(K kind) {
        switch ((AArch64Kind) kind.getPlatformKind()) {
            case BYTE:
            case WORD:
                return kind.changeType(AArch64Kind.DWORD);
            default:
                return kind;
        }
    }

    @Override
    public void emitNullCheck(Value address, LIRFrameState state) {
        append(new AArch64Move.NullCheckOp(asAddressValue(address), state));
    }

    @Override
    public Variable emitAddress(AllocatableValue stackslot) {
        Variable result = newVariable(LIRKind.value(target().arch.getWordKind()));
        append(new AArch64Move.StackLoadAddressOp(result, stackslot));
        return result;
    }

    public AArch64AddressValue asAddressValue(Value address) {
        if (address instanceof AArch64AddressValue) {
            return (AArch64AddressValue) address;
        } else {
            int size = address.getValueKind().getPlatformKind().getSizeInBytes() * Byte.SIZE;
            return AArch64AddressValue.makeAddress(address.getValueKind(), size, asAllocatable(address));
        }
    }

    @Override
    public Variable emitLogicCompareAndSwap(LIRKind accessKind, Value address, Value expectedValue, Value newValue, Value trueValue, Value falseValue, MemoryOrderMode memoryOrder) {
        emitCompareAndSwap(false, accessKind, address, expectedValue, newValue, memoryOrder);
        assert trueValue.getValueKind().equals(falseValue.getValueKind());
        assert isIntConstant(trueValue, 1) && isIntConstant(falseValue, 0);
        Variable result = newVariable(trueValue.getValueKind());
        append(new CondSetOp(result, ConditionFlag.EQ));
        return result;
    }

    @Override
    public Variable emitValueCompareAndSwap(LIRKind accessKind, Value address, Value expectedValue, Value newValue, MemoryOrderMode memoryOrder) {
        return emitCompareAndSwap(true, accessKind, address, expectedValue, newValue, memoryOrder);
    }

    private Variable emitCompareAndSwap(boolean isValue, LIRKind accessKind, Value address, Value expectedValue, Value newValue, MemoryOrderMode memoryOrder) {
        /*
         * Atomic instructions only operate on the general (CPU) registers. Hence, float and double
         * values must temporarily use general registers of the equivalent size.
         */
        LIRKind integerAccessKind = accessKind;
        Value reinterpretedExpectedValue = expectedValue;
        Value reinterpretedNewValue = newValue;
        boolean reinterpretFP = ((AArch64Kind) integerAccessKind.getPlatformKind()).isSIMD();
        if (reinterpretFP) {
            if (accessKind.getPlatformKind().equals(AArch64Kind.SINGLE)) {
                integerAccessKind = LIRKind.fromJavaKind(target().arch, JavaKind.Int);
            } else {
                assert accessKind.getPlatformKind().equals(AArch64Kind.DOUBLE);
                integerAccessKind = LIRKind.fromJavaKind(target().arch, JavaKind.Long);
            }
            reinterpretedExpectedValue = arithmeticLIRGen.emitReinterpret(integerAccessKind, expectedValue);
            reinterpretedNewValue = arithmeticLIRGen.emitReinterpret(integerAccessKind, newValue);
        }
        AArch64Kind memKind = (AArch64Kind) integerAccessKind.getPlatformKind();
        Variable result = newVariable(reinterpretedExpectedValue.getValueKind());
        Variable scratch = newVariable(LIRKind.value(AArch64Kind.DWORD));
        AllocatableValue allocatableExpectedValue = loadReg(reinterpretedExpectedValue);
        AllocatableValue allocatableNewValue = loadReg(reinterpretedNewValue);
        append(new CompareAndSwapOp(memKind, result, allocatableExpectedValue, allocatableNewValue, asAllocatable(address), scratch, memoryOrder));
        if (isValue && reinterpretFP) {
            return asVariable(arithmeticLIRGen.emitReinterpret(accessKind, result));
        } else {
            return result;
        }
    }

    @Override
    public Value emitAtomicReadAndWrite(Value address, ValueKind<?> kind, Value newValue) {
        Variable result = newVariable(kind);
        Variable scratch = newVariable(kind);
        append(new AtomicReadAndWriteOp((AArch64Kind) kind.getPlatformKind(), asAllocatable(result), asAllocatable(address), asAllocatable(newValue), asAllocatable(scratch)));
        return result;
    }

    @Override
    public Value emitAtomicReadAndAdd(Value address, ValueKind<?> kind, Value delta) {
        Variable result = newVariable(kind);
        if (AArch64LIRFlags.useLSE(target().arch)) {
            append(new AtomicReadAndAddLSEOp((AArch64Kind) kind.getPlatformKind(), asAllocatable(result), asAllocatable(address), asAllocatable(delta)));
        } else {
            append(new AtomicReadAndAddOp((AArch64Kind) kind.getPlatformKind(), asAllocatable(result), asAllocatable(address), delta));
        }
        return result;
    }

    @Override
    public void emitMembar(int barriers) {
        int necessaryBarriers = target().arch.requiredBarriers(barriers);
        if (target().isMP && necessaryBarriers != 0) {
            append(new MembarOp(necessaryBarriers));
        }
    }

    @Override
    public void emitJump(LabelRef label) {
        assert label != null;
        append(new StandardOp.JumpOp(label));
    }

    @Override
    public void emitOverflowCheckBranch(LabelRef overflow, LabelRef noOverflow, LIRKind cmpKind, double overflowProbability) {
        append(new AArch64ControlFlow.BranchOp(ConditionFlag.VS, overflow, noOverflow, overflowProbability));
    }

    /**
     * Branches to label if (left & right) == 0. If negated is true branchse on non-zero instead.
     *
     * @param left Integer kind. Non null.
     * @param right Integer kind. Non null.
     * @param trueDestination destination if left & right == 0. Non null.
     * @param falseDestination destination if left & right != 0. Non null
     * @param trueSuccessorProbability hoistoric probability that comparison is true
     */
    @Override
    public void emitIntegerTestBranch(Value left, Value right, LabelRef trueDestination, LabelRef falseDestination, double trueSuccessorProbability) {
        assert ((AArch64Kind) left.getPlatformKind()).isInteger() && left.getPlatformKind() == right.getPlatformKind();
        ((AArch64ArithmeticLIRGenerator) getArithmetic()).emitBinary(AArch64.zr.asValue(LIRKind.combine(left, right)), AArch64ArithmeticOp.TST, true, left, right);
        append(new AArch64ControlFlow.BranchOp(ConditionFlag.EQ, trueDestination, falseDestination, trueSuccessorProbability));
    }

    /**
     * Conditionally move trueValue into new variable if cond + unorderedIsTrue is true, else
     * falseValue.
     *
     * @param left Arbitrary value. Has to have same type as right. Non null.
     * @param right Arbitrary value. Has to have same type as left. Non null.
     * @param cond condition that decides whether to move trueValue or falseValue into result. Non
     *            null.
     * @param unorderedIsTrue defines whether floating-point comparisons consider unordered true or
     *            not. Ignored for integer comparisons.
     * @param trueValue arbitrary value same type as falseValue. Non null.
     * @param falseValue arbitrary value same type as trueValue. Non null.
     * @return value containing trueValue if cond + unorderedIsTrue is true, else falseValue. Non
     *         null.
     */
    @Override
    public Variable emitConditionalMove(PlatformKind cmpKind, Value left, final Value right, Condition cond, boolean unorderedIsTrue, Value trueValue, Value falseValue) {
        AArch64ArithmeticLIRGenerator arithLir = ((AArch64ArithmeticLIRGenerator) arithmeticLIRGen);
        Value actualRight = right;
        if (isJavaConstant(actualRight) && arithLir.mustReplaceNullWithNullRegister((asJavaConstant(actualRight)))) {
            actualRight = arithLir.getNullRegisterValue();
        }
        boolean mirrored = emitCompare(cmpKind, left, actualRight, cond, unorderedIsTrue);
        Condition finalCondition = mirrored ? cond.mirror() : cond;
        boolean finalUnorderedIsTrue = mirrored ? !unorderedIsTrue : unorderedIsTrue;
        ConditionFlag cmpCondition = toConditionFlag(((AArch64Kind) cmpKind).isInteger(), finalCondition, finalUnorderedIsTrue);
        Variable result = newVariable(LIRKind.mergeReferenceInformation(trueValue, falseValue));

        if (isIntConstant(trueValue, 1) && isIntConstant(falseValue, 0)) {
            append(new CondSetOp(result, cmpCondition));
        } else if (isIntConstant(trueValue, 0) && isIntConstant(falseValue, 1)) {
            append(new CondSetOp(result, cmpCondition.negate()));
        } else {
            append(new CondMoveOp(result, cmpCondition, loadReg(trueValue), loadReg(falseValue)));
        }
        return result;
    }

    @Override
    public void emitCompareBranch(PlatformKind cmpKind, Value left, final Value right, Condition cond, boolean unorderedIsTrue, LabelRef trueDestination, LabelRef falseDestination,
                    double trueDestinationProbability) {
        Value actualRight = right;
        if (cond == Condition.EQ) {
            // emit cbz instruction for IsNullNode.
            assert !LIRValueUtil.isNullConstant(left) : "emitNullCheckBranch()'s null input should be in right.";
            AArch64ArithmeticLIRGenerator arithLir = ((AArch64ArithmeticLIRGenerator) arithmeticLIRGen);
            if (LIRValueUtil.isNullConstant(actualRight)) {
                JavaConstant rightConstant = asJavaConstant(actualRight);
                if (arithLir.mustReplaceNullWithNullRegister(rightConstant)) {
                    actualRight = arithLir.getNullRegisterValue();
                } else {
                    append(new CompareBranchZeroOp(asAllocatable(left), trueDestination, falseDestination,
                                    trueDestinationProbability));
                    return;
                }
            }

            // emit cbz instruction for IntegerEquals when any of the inputs is zero.
            AArch64Kind kind = (AArch64Kind) cmpKind;
            if (kind.isInteger()) {
                if (isIntConstant(left, 0)) {
                    append(new CompareBranchZeroOp(asAllocatable(actualRight), trueDestination, falseDestination, trueDestinationProbability));
                    return;
                } else if (isIntConstant(actualRight, 0)) {
                    append(new CompareBranchZeroOp(asAllocatable(left), trueDestination, falseDestination, trueDestinationProbability));
                    return;
                }
            }
        }

        boolean mirrored = emitCompare(cmpKind, left, actualRight, cond, unorderedIsTrue);
        Condition finalCondition = mirrored ? cond.mirror() : cond;
        boolean finalUnorderedIsTrue = mirrored ? !unorderedIsTrue : unorderedIsTrue;
        ConditionFlag cmpCondition = toConditionFlag(((AArch64Kind) cmpKind).isInteger(), finalCondition, finalUnorderedIsTrue);
        append(new BranchOp(cmpCondition, trueDestination, falseDestination, trueDestinationProbability));
    }

    private static ConditionFlag toConditionFlag(boolean isInt, Condition cond, boolean unorderedIsTrue) {
        return isInt ? toIntConditionFlag(cond) : toFloatConditionFlag(cond, unorderedIsTrue);
    }

    /**
     * Takes a Condition and unorderedIsTrue flag and returns the correct Aarch64 specific
     * ConditionFlag. Note: This is only correct if the emitCompare code for floats has correctly
     * handled the case of 'EQ && unorderedIsTrue', respectively 'NE && !unorderedIsTrue'!
     */
    private static ConditionFlag toFloatConditionFlag(Condition cond, boolean unorderedIsTrue) {
        switch (cond) {
            case LT:
                return unorderedIsTrue ? ConditionFlag.LT : ConditionFlag.LO;
            case LE:
                return unorderedIsTrue ? ConditionFlag.LE : ConditionFlag.LS;
            case GE:
                return unorderedIsTrue ? ConditionFlag.PL : ConditionFlag.GE;
            case GT:
                return unorderedIsTrue ? ConditionFlag.HI : ConditionFlag.GT;
            case EQ:
                return ConditionFlag.EQ;
            case NE:
                return ConditionFlag.NE;
            default:
                throw GraalError.shouldNotReachHere();
        }
    }

    /**
     * Takes a Condition and returns the correct Aarch64 specific ConditionFlag.
     */
    private static ConditionFlag toIntConditionFlag(Condition cond) {
        switch (cond) {
            case EQ:
                return ConditionFlag.EQ;
            case NE:
                return ConditionFlag.NE;
            case LT:
                return ConditionFlag.LT;
            case LE:
                return ConditionFlag.LE;
            case GT:
                return ConditionFlag.GT;
            case GE:
                return ConditionFlag.GE;
            case AE:
                return ConditionFlag.HS;
            case BE:
                return ConditionFlag.LS;
            case AT:
                return ConditionFlag.HI;
            case BT:
                return ConditionFlag.LO;
            default:
                throw GraalError.shouldNotReachHere();
        }
    }

    /**
     * This method emits the compare instruction, and may reorder the operands. It returns true if
     * it did so.
     *
     * @param a the left operand of the comparison. Has to have same type as b. Non null.
     * @param b the right operand of the comparison. Has to have same type as a. Non null.
     * @return true if mirrored (i.e. "b cmp a" instead of "a cmp b" was done).
     */
    protected boolean emitCompare(PlatformKind cmpKind, Value a, Value b, Condition condition, boolean unorderedIsTrue) {
        Value left;
        Value right;
        boolean mirrored;
        AArch64Kind kind = (AArch64Kind) cmpKind;
        if (kind.isInteger()) {
            Value aExt = a;
            Value bExt = b;

            assert a.getPlatformKind() == b.getPlatformKind();
            /*
             * AArch64 compares 32 or 64 bits: sign extend a and b as required. Note currently the
             * size of the comparison within AArch64Compare is based on the size of the operands,
             * not the comparison size provided here.
             */
            int compareBits = cmpKind.getSizeInBytes() * Byte.SIZE;
            int operandBits = a.getPlatformKind().getSizeInBytes() * Byte.SIZE;
            switch (compareBits) {
                case 8:
                case 16:
                    /* Lower bits need to be sign extended. */
                    aExt = arithmeticLIRGen.emitSignExtend(a, compareBits, 64);
                    bExt = arithmeticLIRGen.emitSignExtend(b, compareBits, 64);
                    break;
                case 32:
                case 64:
                    /*
                     * May need to extend operands to be at least 32 bits. If both operands are
                     * 32-bits, it is unnecessary to extend them to 64 bits, even if compareBits is
                     * 64.
                     */
                    if (operandBits < 32) {
                        aExt = arithmeticLIRGen.emitSignExtend(a, operandBits, compareBits);
                        bExt = arithmeticLIRGen.emitSignExtend(b, operandBits, compareBits);
                    }
                    break;
                default:
                    throw GraalError.shouldNotReachHere();

            }

            /*
             * The AArch64 comparison instruction can treat register 31 as the stack pointer
             * register (sp) for the left operand, but not for the right operand.
             */
            boolean aIsStackPointer = ValueUtil.isRegister(aExt) && ValueUtil.asRegister(aExt).equals(AArch64.sp);
            boolean bIsStackPointer = ValueUtil.isRegister(bExt) && ValueUtil.asRegister(bExt).equals(AArch64.sp);

            if (aIsStackPointer && bIsStackPointer) {
                /*
                 * If both a and b are sp, this cannot be encoded in an AArch64 comparison. Hence,
                 * sp must be moved to a register.
                 */
                left = right = emitMove(aExt);
                mirrored = false;
            } else if (bIsStackPointer || (isCompareConstant(aExt) && !isCompareConstant(bExt))) {
                left = bExt;
                right = loadNonConst(aExt);
                mirrored = true;
            } else {
                left = aExt;
                right = loadNonConst(bExt);
                mirrored = false;
            }
            append(new AArch64Compare.CompareOp(loadReg(left), loadNonCompareConst(right)));
        } else if (kind.isSIMD()) {
            if (AArch64Compare.FloatCompareOp.isFloatCmpConstant(a, condition, unorderedIsTrue)) {
                left = b;
                right = a;
                mirrored = true;
            } else if (AArch64Compare.FloatCompareOp.isFloatCmpConstant(b, condition, unorderedIsTrue)) {
                left = a;
                right = b;
                mirrored = false;
            } else {
                left = a;
                right = loadReg(b);
                mirrored = false;
            }
            append(new AArch64Compare.FloatCompareOp(loadReg(left), right, condition, unorderedIsTrue));
        } else {
            throw GraalError.shouldNotReachHere();
        }
        return mirrored;
    }

    /**
     * If value is a constant that cannot be used directly with a gpCompare instruction load it into
     * a register and return the register, otherwise return constant value unchanged.
     */
    protected Value loadNonCompareConst(Value value) {
        if (!isCompareConstant(value)) {
            return loadReg(value);
        }
        return value;
    }

    /**
     * Checks whether value can be used directly with a gpCompare instruction. This is <b>not</b>
     * the same as {@link AArch64ArithmeticLIRGenerator#isArithmeticConstant(JavaConstant)}, because
     * 0.0 is a valid compare constant for floats, while there are no arithmetic constants for
     * floats.
     *
     * @param value any type. Non null.
     * @return true if value can be used directly in comparison instruction, false otherwise.
     */
    public boolean isCompareConstant(Value value) {
        if (isJavaConstant(value)) {
            JavaConstant constant = asJavaConstant(value);
            if (constant instanceof PrimitiveConstant) {
                final long longValue = constant.asLong();
                long maskedValue;
                switch (constant.getJavaKind()) {
                    case Boolean:
                    case Byte:
                        maskedValue = longValue & 0xFF;
                        break;
                    case Char:
                    case Short:
                        maskedValue = longValue & 0xFFFF;
                        break;
                    case Int:
                        maskedValue = longValue & 0xFFFF_FFFFL;
                        break;
                    case Long:
                        maskedValue = longValue;
                        break;
                    default:
                        throw GraalError.shouldNotReachHere();
                }
                return AArch64MacroAssembler.isArithmeticImmediate(maskedValue);
            } else {
                return constant.isDefaultForKind();
            }
        }
        return false;
    }

    /**
     * Moves trueValue into result if (left & right) == 0, else falseValue.
     *
     * @param left Integer kind. Non null.
     * @param right Integer kind. Non null.
     * @param trueValue Integer kind. Non null.
     * @param falseValue Integer kind. Non null.
     * @return virtual register containing trueValue if (left & right) == 0, else falseValue.
     */
    @Override
    public Variable emitIntegerTestMove(Value left, Value right, Value trueValue, Value falseValue) {
        assert ((AArch64Kind) left.getPlatformKind()).isInteger() && ((AArch64Kind) right.getPlatformKind()).isInteger();
        assert ((AArch64Kind) trueValue.getPlatformKind()).isInteger() && ((AArch64Kind) falseValue.getPlatformKind()).isInteger();
        ((AArch64ArithmeticLIRGenerator) getArithmetic()).emitBinary(AArch64.zr.asValue(LIRKind.combine(left, right)), AArch64ArithmeticOp.TST, true, left, right);
        Variable result = newVariable(LIRKind.mergeReferenceInformation(trueValue, falseValue));

        if (isIntConstant(trueValue, 1) && isIntConstant(falseValue, 0)) {
            append(new CondSetOp(result, ConditionFlag.EQ));
        } else if (isIntConstant(trueValue, 0) && isIntConstant(falseValue, 1)) {
            append(new CondSetOp(result, ConditionFlag.NE));
        } else {
            append(new CondMoveOp(result, ConditionFlag.EQ, load(trueValue), load(falseValue)));
        }
        return result;
    }

    @Override
    public void emitStrategySwitch(SwitchStrategy strategy, Variable key, LabelRef[] keyTargets, LabelRef defaultTarget) {
        append(createStrategySwitchOp(strategy, keyTargets, defaultTarget, key, newVariable(key.getValueKind()), AArch64LIRGenerator::toIntConditionFlag));
    }

    protected StrategySwitchOp createStrategySwitchOp(SwitchStrategy strategy, LabelRef[] keyTargets, LabelRef defaultTarget, Variable key, AllocatableValue scratchValue,
                    Function<Condition, ConditionFlag> converter) {
        return new StrategySwitchOp(strategy, keyTargets, defaultTarget, key, scratchValue, converter);
    }

    @Override
    protected void emitTableSwitch(int lowKey, LabelRef defaultTarget, LabelRef[] targets, Value key) {
        append(new TableSwitchOp(lowKey, defaultTarget, targets, key, newVariable(LIRKind.value(target().arch.getWordKind())), newVariable(key.getValueKind())));
    }

    @Override
    public Variable emitByteSwap(Value input) {
        Variable result = newVariable(LIRKind.combine(input));
        append(new AArch64ByteSwap.ByteSwapOp(result, asAllocatable(input)));
        return result;
    }

    @Override
    public Variable emitArrayCompareTo(JavaKind kind1, JavaKind kind2, int array1BaseOffset, int array2BaseOffset, Value array1, Value array2, Value length1, Value length2) {
        LIRKind resultKind = LIRKind.value(AArch64Kind.DWORD);
        // DMS TODO: check calling conversion and registers used
        RegisterValue res = AArch64.r0.asValue(resultKind);
        RegisterValue cnt1 = AArch64.r1.asValue(length1.getValueKind());
        RegisterValue cnt2 = AArch64.r2.asValue(length2.getValueKind());
        emitMove(cnt1, length1);
        emitMove(cnt2, length2);
        append(new AArch64ArrayCompareToOp(this, kind1, kind2, array1BaseOffset, array2BaseOffset, res, array1, array2, cnt1, cnt2));
        Variable result = newVariable(resultKind);
        emitMove(result, res);
        return result;
    }

    @Override
    public Variable emitArrayEquals(JavaKind kind, int array1BaseOffset, int array2BaseOffset, Value array1, Value array2, Value length, boolean directPointers) {
        Variable result = newVariable(LIRKind.value(AArch64Kind.DWORD));
        append(new AArch64ArrayEqualsOp(this, kind, array1BaseOffset, array2BaseOffset, result, array1, array2, asAllocatable(length), directPointers));
        return result;
    }

    @Override
    public Variable emitArrayIndexOf(int arrayBaseOffset, JavaKind valueKind, boolean findTwoConsecutive, Value arrayPointer, Value arrayLength, Value fromIndex, Value... searchValues) {
        assert searchValues.length == 1;
        Variable result = newVariable(LIRKind.value(AArch64Kind.DWORD));
        append(new AArch64ArrayIndexOfOp(arrayBaseOffset, valueKind, this, result, asAllocatable(arrayPointer), asAllocatable(arrayLength), asAllocatable(fromIndex), asAllocatable(searchValues[0])));
        return result;
    }

    @Override
    protected JavaConstant zapValueForKind(PlatformKind kind) {
        long dead = 0xDEADDEADDEADDEADL;
        switch ((AArch64Kind) kind) {
            case BYTE:
                return JavaConstant.forByte((byte) dead);
            case WORD:
                return JavaConstant.forShort((short) dead);
            case DWORD:
                return JavaConstant.forInt((int) dead);
            case QWORD:
                return JavaConstant.forLong(dead);
            case SINGLE:
                return JavaConstant.forFloat(Float.intBitsToFloat((int) dead));
            case DOUBLE:
                return JavaConstant.forDouble(Double.longBitsToDouble(dead));
            default:
                throw GraalError.shouldNotReachHere();
        }
    }

    /**
     * Loads value into virtual register. Contrary to {@link #load(Value)} this handles
     * RegisterValues (i.e. values corresponding to fixed physical registers) correctly, by not
     * creating an unnecessary move into a virtual register.
     *
     * This avoids generating the following code:
     *
     * <pre>
     * mov x0, x19 # x19 is fixed thread register
     * ldr x0, [x0]
     * </pre>
     *
     * instead of:
     *
     * <pre>
     * ldr x0, [x19]
     * </pre>
     */
    protected AllocatableValue loadReg(Value val) {
        if (!(LIRValueUtil.isVariable(val) || val instanceof RegisterValue)) {
            return emitMove(val);
        }
        return (AllocatableValue) val;
    }

    @Override
    public void emitPause() {
        append(new AArch64PauseOp());
    }

    @Override
    public void emitCacheWriteback(Value address) {
        append(new AArch64CacheWritebackOp(asAddressValue(address)));
    }

    @Override
    public void emitCacheWritebackSync(boolean isPreSync) {
        // only need a post sync barrier on AArch64
        if (!isPreSync) {
            append(new AArch64CacheWritebackPostSyncOp());
        }
    }

    @Override
    public LIRInstruction createZapRegisters(Register[] zappedRegisters, JavaConstant[] zapValues) {
        return new AArch64ZapRegistersOp(zappedRegisters, zapValues);
    }

    @Override
    public LIRInstruction createZapArgumentSpace(StackSlot[] zappedStack, JavaConstant[] zapValues) {
        return new AArch64ZapStackOp(zappedStack, zapValues);
    }

    public abstract void emitCCall(long address, CallingConvention nativeCallingConvention, Value[] args);

    @Override
    public void emitSpeculationFence() {
        append(new AArch64SpeculativeBarrier());
    }

    @Override
    public void emitZeroMemory(Value address, Value length, boolean isAligned) {
        emitZeroMemory(address, length, isAligned, false, -1);
    }

    protected final void emitZeroMemory(Value address, Value length, boolean isAligned, boolean useDcZva, int zvaLength) {
        RegisterValue regAddress = AArch64.r0.asValue(address.getValueKind());
        RegisterValue regLength = AArch64.r1.asValue(length.getValueKind());
        emitMove(regAddress, address);
        emitMove(regLength, length);
        append(new AArch64ZeroMemoryOp(regAddress, regLength, isAligned, useDcZva, zvaLength));
    }
}
