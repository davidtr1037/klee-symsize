//===-- ExecutionState.cpp ------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "ExecutionState.h"

#include "Memory.h"

#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Module/Cell.h"
#include "klee/Module/InstructionInfoTable.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Support/OptionCategories.h"

#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

#include <cassert>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdarg.h>

using namespace llvm;
using namespace klee;

namespace {
cl::opt<bool> DebugLogStateMerge(
    "debug-log-state-merge", cl::init(false),
    cl::desc("Debug information for underlying state merging (default=false)"),
    cl::cat(MergeCat));
}

/***/

std::uint32_t ExecutionState::nextID = 1;

/***/

StackFrame::StackFrame(KInstIterator _caller, KFunction *_kf)
  : caller(_caller), kf(_kf), callPathNode(0), 
    minDistToUncoveredOnReturn(0), varargs(0), isExecutingLoop(false), loop(nullptr) {
  locals = new Cell[kf->numRegisters];
}

StackFrame::StackFrame(const StackFrame &s) 
  : caller(s.caller),
    kf(s.kf),
    callPathNode(s.callPathNode),
    allocas(s.allocas),
    minDistToUncoveredOnReturn(s.minDistToUncoveredOnReturn),
    varargs(s.varargs),
    isExecutingLoop(s.isExecutingLoop),
    loop(s.loop) {
  locals = new Cell[s.kf->numRegisters];
  for (unsigned i=0; i<s.kf->numRegisters; i++)
    locals[i] = s.locals[i];
}

StackFrame::~StackFrame() { 
  delete[] locals; 
}

/***/

ExecutionState::ExecutionState(KFunction *kf) :
    pc(kf->instructions),
    prevPC(pc),
    depth(0),
    ptreeNode(nullptr),
    steppedInstructions(0),
    instsSinceCovNew(0),
    coveredNew(false),
    forkDisabled(false),
    loopHandler(nullptr) {
  pushFrame(nullptr, kf);
  setID();
}

ExecutionState::~ExecutionState() {
  for (const auto &cur_mergehandler: openMergeStack){
    cur_mergehandler->removeOpenState(this);
  }
  for (ref<LoopHandler> handler : openLoopHandlerStack){
    handler->removeOpenState(this);
  }
  if (!loopHandler.isNull()) {
    /* TODO: mark here as incomplete execution? */
    loopHandler->removeOpenState(this);
  }

  while (!stack.empty()) popFrame();
}

ExecutionState::ExecutionState(const ExecutionState& state):
    pc(state.pc),
    prevPC(state.prevPC),
    stack(state.stack),
    incomingBBIndex(state.incomingBBIndex),
    depth(state.depth),
    addressSpace(state.addressSpace),
    constraints(state.constraints),
    pathOS(state.pathOS),
    symPathOS(state.symPathOS),
    coveredLines(state.coveredLines),
    symbolics(state.symbolics),
    arrayNames(state.arrayNames),
    openMergeStack(state.openMergeStack),
    openLoopHandlerStack(state.openLoopHandlerStack),
    steppedInstructions(state.steppedInstructions),
    instsSinceCovNew(state.instsSinceCovNew),
    coveredNew(state.coveredNew),
    forkDisabled(state.forkDisabled),
    /* TODO: copy-on-write? */
    taintedExprs(state.taintedExprs),
    loopHandler(state.loopHandler),
    suffixConstraints(state.suffixConstraints) {
  for (const auto &cur_mergehandler: openMergeStack) {
    cur_mergehandler->addOpenState(this);
  }
  for (ref<LoopHandler> handler : openLoopHandlerStack) {
    handler->addOpenState(this);
  }
  if (!loopHandler.isNull()) {
    loopHandler->addOpenState(this);
  }
}

ExecutionState *ExecutionState::branch() {
  depth++;

  auto *falseState = new ExecutionState(*this);
  falseState->setID();
  falseState->coveredNew = false;
  falseState->coveredLines.clear();

  return falseState;
}

void ExecutionState::pushFrame(KInstIterator caller, KFunction *kf) {
  stack.emplace_back(StackFrame(caller, kf));
}

void ExecutionState::popFrame() {
  const StackFrame &sf = stack.back();
  for (const auto * memoryObject : sf.allocas)
    addressSpace.unbindObject(memoryObject);
  stack.pop_back();
}

void ExecutionState::addSymbolic(const MemoryObject *mo, const Array *array) {
  symbolics.emplace_back(ref<const MemoryObject>(mo), array);
}

/**/

llvm::raw_ostream &klee::operator<<(llvm::raw_ostream &os, const MemoryMap &mm) {
  os << "{";
  MemoryMap::iterator it = mm.begin();
  MemoryMap::iterator ie = mm.end();
  if (it!=ie) {
    os << "MO" << it->first->id << ":" << it->second.get();
    for (++it; it!=ie; ++it)
      os << ", MO" << it->first->id << ":" << it->second.get();
  }
  os << "}";
  return os;
}

bool ExecutionState::merge(const ExecutionState &b) {
  if (DebugLogStateMerge)
    llvm::errs() << "-- attempting merge of A:" << this << " with B:" << &b
                 << "--\n";
  if (pc != b.pc)
    return false;

  // XXX is it even possible for these to differ? does it matter? probably
  // implies difference in object states?

  if (symbolics != b.symbolics)
    return false;

  {
    std::vector<StackFrame>::const_iterator itA = stack.begin();
    std::vector<StackFrame>::const_iterator itB = b.stack.begin();
    while (itA!=stack.end() && itB!=b.stack.end()) {
      // XXX vaargs?
      if (itA->caller!=itB->caller || itA->kf!=itB->kf)
        return false;
      ++itA;
      ++itB;
    }
    if (itA!=stack.end() || itB!=b.stack.end())
      return false;
  }

  std::set< ref<Expr> > aConstraints(constraints.begin(), constraints.end());
  std::set< ref<Expr> > bConstraints(b.constraints.begin(), 
                                     b.constraints.end());
  std::set< ref<Expr> > commonConstraints, aSuffix, bSuffix;
  std::set_intersection(aConstraints.begin(), aConstraints.end(),
                        bConstraints.begin(), bConstraints.end(),
                        std::inserter(commonConstraints, commonConstraints.begin()));
  std::set_difference(aConstraints.begin(), aConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(aSuffix, aSuffix.end()));
  std::set_difference(bConstraints.begin(), bConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(bSuffix, bSuffix.end()));
  if (DebugLogStateMerge) {
    llvm::errs() << "\tconstraint prefix: [";
    for (std::set<ref<Expr> >::iterator it = commonConstraints.begin(),
                                        ie = commonConstraints.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tA suffix: [";
    for (std::set<ref<Expr> >::iterator it = aSuffix.begin(),
                                        ie = aSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tB suffix: [";
    for (std::set<ref<Expr> >::iterator it = bSuffix.begin(),
                                        ie = bSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
  }

  // We cannot merge if addresses would resolve differently in the
  // states. This means:
  // 
  // 1. Any objects created since the branch in either object must
  // have been free'd.
  //
  // 2. We cannot have free'd any pre-existing object in one state
  // and not the other

  if (DebugLogStateMerge) {
    llvm::errs() << "\tchecking object states\n";
    llvm::errs() << "A: " << addressSpace.objects << "\n";
    llvm::errs() << "B: " << b.addressSpace.objects << "\n";
  }
    
  std::set<const MemoryObject*> mutated;
  MemoryMap::iterator ai = addressSpace.objects.begin();
  MemoryMap::iterator bi = b.addressSpace.objects.begin();
  MemoryMap::iterator ae = addressSpace.objects.end();
  MemoryMap::iterator be = b.addressSpace.objects.end();
  for (; ai!=ae && bi!=be; ++ai, ++bi) {
    if (ai->first != bi->first) {
      if (DebugLogStateMerge) {
        if (ai->first < bi->first) {
          llvm::errs() << "\t\tB misses binding for: " << ai->first->id << "\n";
        } else {
          llvm::errs() << "\t\tA misses binding for: " << bi->first->id << "\n";
        }
      }
      return false;
    }
    if (ai->second.get() != bi->second.get()) {
      if (DebugLogStateMerge)
        llvm::errs() << "\t\tmutated: " << ai->first->id << "\n";
      mutated.insert(ai->first);
    }
  }
  if (ai!=ae || bi!=be) {
    if (DebugLogStateMerge)
      llvm::errs() << "\t\tmappings differ\n";
    return false;
  }
  
  // merge stack

  ref<Expr> inA = ConstantExpr::alloc(1, Expr::Bool);
  ref<Expr> inB = ConstantExpr::alloc(1, Expr::Bool);
  for (std::set< ref<Expr> >::iterator it = aSuffix.begin(), 
         ie = aSuffix.end(); it != ie; ++it)
    inA = AndExpr::create(inA, *it);
  for (std::set< ref<Expr> >::iterator it = bSuffix.begin(), 
         ie = bSuffix.end(); it != ie; ++it)
    inB = AndExpr::create(inB, *it);

  // XXX should we have a preference as to which predicate to use?
  // it seems like it can make a difference, even though logically
  // they must contradict each other and so inA => !inB

  std::vector<StackFrame>::iterator itA = stack.begin();
  std::vector<StackFrame>::const_iterator itB = b.stack.begin();
  for (; itA!=stack.end(); ++itA, ++itB) {
    StackFrame &af = *itA;
    const StackFrame &bf = *itB;
    for (unsigned i=0; i<af.kf->numRegisters; i++) {
      ref<Expr> &av = af.locals[i].value;
      const ref<Expr> &bv = bf.locals[i].value;
      if (av.isNull() || bv.isNull()) {
        // if one is null then by implication (we are at same pc)
        // we cannot reuse this local, so just ignore
      } else {
        av = SelectExpr::create(inA, av, bv);
      }
    }
  }

  for (std::set<const MemoryObject*>::iterator it = mutated.begin(), 
         ie = mutated.end(); it != ie; ++it) {
    const MemoryObject *mo = *it;
    const ObjectState *os = addressSpace.findObject(mo);
    const ObjectState *otherOS = b.addressSpace.findObject(mo);
    assert(os && !os->readOnly && 
           "objects mutated but not writable in merging state");
    assert(otherOS);
    assert(os->getObject()->capacity == otherOS->getObject()->capacity);

    ObjectState *wos = addressSpace.getWriteable(mo, os);
    for (unsigned i = 0; i < mo->capacity; i++) {
      ref<Expr> av = wos->read8(i);
      ref<Expr> bv = otherOS->read8(i);
      ref<Expr> cv = SelectExpr::create(inA, av, bv);
      wos->write(i, cv);
    }
  }

  constraints = ConstraintSet();

  ConstraintManager m(constraints);
  for (const auto &constraint : commonConstraints)
    m.addConstraint(constraint);
  m.addConstraint(OrExpr::create(inA, inB));

  if (DebugLogStateMerge) {
    llvm::errs() << "merge succeeded\n";
  }
  return true;
}

void ExecutionState::dumpStack(llvm::raw_ostream &out) const {
  unsigned idx = 0;
  const KInstruction *target = prevPC;
  for (ExecutionState::stack_ty::const_reverse_iterator
         it = stack.rbegin(), ie = stack.rend();
       it != ie; ++it) {
    const StackFrame &sf = *it;
    Function *f = sf.kf->function;
    const InstructionInfo &ii = *target->info;
    out << "\t#" << idx++;
    std::stringstream AssStream;
    AssStream << std::setw(8) << std::setfill('0') << ii.assemblyLine;
    out << AssStream.str();
    out << " in " << f->getName().str() << " (";
    // Yawn, we could go up and print varargs if we wanted to.
    unsigned index = 0;
    for (Function::arg_iterator ai = f->arg_begin(), ae = f->arg_end();
         ai != ae; ++ai) {
      if (ai!=f->arg_begin()) out << ", ";

      out << ai->getName().str();
      // XXX should go through function
      ref<Expr> value = sf.locals[sf.kf->getArgRegister(index++)].value;
      if (value.get() && isa<ConstantExpr>(value))
        out << "=" << value;
    }
    out << ")";
    if (ii.file != "")
      out << " at " << ii.file << ":" << ii.line;
    out << "\n";
    target = sf.caller;
  }
}

void ExecutionState::addConstraint(ref<Expr> e) {
  ConstraintManager c(constraints);
  c.addConstraint(e);
  if (!loopHandler.isNull()) {
    /* we don't expect forks after the state is paused */
    assert(stack.back().isExecutingLoop);
    ConstraintManager m(suffixConstraints);
    m.addConstraint(e);
  }
}

void ExecutionState::addTaintedExpr(std::string name, ref<Expr> offset) {
  ExprSet &exprs = taintedExprs[name];
  for (ref<Expr> e : exprs) {
    if (*e == *offset) {
      return;
    }
  }

  exprs.push_back(offset);
}

/* TODO: handle symbolic offsets... */
bool ExecutionState::hasTaintedExpr(std::string name, ref<Expr> offset) {
  auto i = taintedExprs.find(name);
  if (i != taintedExprs.end()) {
    for (ref<Expr> e : i->second) {
      /* TODO: this is a conservative check, use the solver? */
      if (!isa<ConstantExpr>(offset) || !isa<ConstantExpr>(e) || *e == *offset) {
        return true;
      }
    }
  }

  return false;
}

bool ExecutionState::isTaintedExpr(ref<Expr> e) {
  TaintVisitor visitor(*this);
  visitor.visit(e);
  return visitor.isTainted;
}

ExprVisitor::Action TaintVisitor::visitRead(const ReadExpr &e) {
  for (const UpdateNode *un = e.updates.head.get(); un != nullptr; un = un->next.get()) {
    visit(un->index);
    visit(un->value);
  }

  const std::string &name = e.updates.root->getName();
  if (state.hasTaintedExpr(name, e.index)) {
    isTainted = true;
    return Action::skipChildren();
  } else {
    return Action::doChildren();
  }
}

ExecutionContext::ExecutionContext(ExecutionState &state) {
  const KInstruction *kinst = state.prevPC;
  for (auto i = state.stack.rbegin(); i != state.stack.rend(); i++) {
    StackFrame &sf = *i;
    Function *f = sf.kf->function;
    if (f->getName() == "__uClibc_main") {
      /* not interesting from that point on... */
      break;
    }

    CodeLocation location(kinst ? kinst->info->file : "unknown",
                          kinst ? kinst->info->line : 0,
                          f->getName());
    trace.push_back(location);
    kinst = sf.caller;
  }
}

void ExecutionContext::dump() const {
  for (const CodeLocation &location : trace) {
    location.dump();
  }
}

ExecutionState *ExecutionState::mergeStates(std::vector<ExecutionState *> &states) {
  assert(!states.empty());
  ExecutionState *merged = states[0];

  for (unsigned i = 1; i < states.size(); i++) {
    ExecutionState *es = states[i];
    if (!merged->merge(*es)) {
      return nullptr;
    }
  }

  return merged;
}

ExecutionState *ExecutionState::mergeStatesOptimized(std::vector<ExecutionState *> &states,
                                                     bool isComplete,
                                                     LoopHandler *loopHandler) {
  assert(!states.empty());
  ExecutionState *merged = states[0];

  /* program counter */
  for (ExecutionState *es : states) {
    if (es->pc != merged->pc) {
      return nullptr;
    }
  }

  /* symbolics */
  for (ExecutionState *es : states) {
    if (es->symbolics != merged->symbolics) {
      return nullptr;
    }
  }

  /* stack */
  for (ExecutionState *es : states) {
    auto i = merged->stack.begin();
    auto j = es->stack.begin();
    while (i != merged->stack.end() && j != es->stack.end()) {
      if (i->caller != j->caller || i->kf != j->kf) {
        return nullptr;
      }
      ++i;
      ++j;
    }
    if (i != merged->stack.end() || j != es->stack.end()) {
      return nullptr;
    }
  }

  /* address space */
  std::set<const MemoryObject*> mutated;
  for (ExecutionState *es : states) {
    auto ai = merged->addressSpace.objects.begin();
    auto bi = es->addressSpace.objects.begin();
    auto ae = merged->addressSpace.objects.end();
    auto be = es->addressSpace.objects.end();
    for (; ai != ae && bi != be; ++ai, ++bi) {
      if (ai->first != bi->first) {
        return nullptr;
      }
      if (ai->second.get() != bi->second.get()) {
        mutated.insert(ai->first);
      }
    }
    if (ai != ae || bi != be) {
      return nullptr;
    }
  }

  /* compute suffix for each state */
  std::vector<ref<Expr>> suffixes;
  for (unsigned i = 0; i < states.size(); i++) {
    ExecutionState *es = states[i];
    ref<Expr> all = ConstantExpr::create(1, Expr::Bool);
    for (ref<Expr> e : es->suffixConstraints) {
      all = AndExpr::create(all, e);
    }
    suffixes.push_back(all);
  }

  /* local vars */
  for (unsigned i = 0; i < merged->stack.size(); i++) {
    StackFrame &sf = merged->stack[i];
    for (unsigned reg = 0; reg < sf.kf->numRegisters; reg++) {
      bool ignore = false;
      for (ExecutionState *es : states) {
        ref<Expr> v = es->stack[i].locals[reg].value;
        if (v.isNull()) {
          ignore = true;
          break;
        }
      }
      if (ignore) {
        continue;
      }

      std::vector<ref<Expr>> values;
      for (ExecutionState *es : states) {
        values.push_back(es->stack[i].locals[reg].value);
      }
      ref<Expr> &v = sf.locals[reg].value;
      v = mergeValues(suffixes, values);
    }
  }

  /* heap */
  for (auto it = mutated.begin(), ie = mutated.end(); it != ie; ++it) {
    const MemoryObject *mo = *it;
    const ObjectState *os = merged->addressSpace.findObject(mo);
    assert(os && !os->readOnly && "objects mutated but not writable in merging state");
    ObjectState *wos = merged->addressSpace.getWriteable(mo, os);

    for (unsigned i = 0; i < mo->capacity; i++) {
      std::vector<ref<Expr>> values;
      for (ExecutionState *es : states) {
        const ObjectState *other = es->addressSpace.findObject(mo);
        assert(other);
        assert(wos->getObject()->capacity == other->getObject()->capacity);
        values.push_back(other->read8(i));
      }
      ref<Expr> v = mergeValues(suffixes, values);
      wos->write(i, v);
    }
  }

  /* path constraints */
  merged->constraints = ConstraintSet();
  ConstraintManager m(merged->constraints);
  for (ref<Expr> e : loopHandler->initialConstraints) {
    /* add without the manager? (the prefix is already optimized) */
    m.addConstraint(e);
  }

  if (!isComplete) {
    ref<Expr> orExpr = ConstantExpr::create(0, Expr::Bool);
    for (ExecutionState *es : states) {
      /* build suffix conjunction */
      ref<Expr> andExpr = ConstantExpr::create(1, Expr::Bool);
      for (ref<Expr> e : es->suffixConstraints) {
        andExpr = AndExpr::create(andExpr, e);
      }

      /* update disjunction */
      orExpr = OrExpr::create(orExpr, andExpr);
    }

    /* TODO: used ExecutionState's addConstraint? */
    m.addConstraint(orExpr);
  }

  return merged;
}

ref<Expr> ExecutionState::mergeValues(std::vector<ref<Expr>> &suffixes,
                                      std::vector<ref<Expr>> &values) {
  assert(suffixes.size() == values.size());

  ref<Expr> summary = values[0];
  for (unsigned j = 1; j < values.size(); j++) {
    ref<Expr> e = values[j];
    ref<Expr> cond = suffixes[j - 1];
    summary = SelectExpr::create(cond, summary, e);
  }

  return summary;
}
