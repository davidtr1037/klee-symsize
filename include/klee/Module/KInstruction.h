//===-- KInstruction.h ------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_KINSTRUCTION_H
#define KLEE_KINSTRUCTION_H

#include "klee/Config/Version.h"
#include "klee/Module/InstructionInfoTable.h"
#include "klee/Module/KLoop.h"

#include "llvm/Support/DataTypes.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Analysis/LoopInfo.h"

#include <vector>

namespace llvm {
  class Instruction;
}

namespace klee {
  class Executor;
  struct InstructionInfo;
  class KModule;


  /// KInstruction - Intermediate instruction representation used
  /// during execution.
  struct KInstruction {
    llvm::Instruction *inst;    
    const InstructionInfo *info;

    /// Value numbers for each operand. -1 is an invalid value,
    /// otherwise negative numbers are indices (negated and offset by
    /// 2) into the module constant table and positive numbers are
    /// register indices.
    int *operands;
    /// Destination register index.
    unsigned dest;
    /* TODO: use pointer? */
    KLoop entryLoop;
    /* TODO: use pointer? */
    std::vector<KLoop> exitLoops;
    /* TODO: add docs */
    bool isLoopEntry;
    /* TODO: add docs */
    bool isLoopExit;

  public:
    KInstruction();
    virtual ~KInstruction();
    std::string getSourceLocation() const;

    bool hasExitLoop(llvm::Loop *loop) {
      for (const KLoop &kloop : exitLoops) {
        if (kloop.loop == loop) {
          return true;
        }
      }
      return false;
    }

  };

  struct KGEPInstruction : KInstruction {
    /// indices - The list of variable sized adjustments to add to the pointer
    /// operand to execute the instruction. The first element is the operand
    /// index into the GetElementPtr instruction, and the second element is the
    /// element size to multiple that index by.
    std::vector< std::pair<unsigned, uint64_t> > indices;

    /// offset - A constant offset to add to the pointer operand to execute the
    /// instruction.
    uint64_t offset;
  };
}

#endif /* KLEE_KINSTRUCTION_H */
