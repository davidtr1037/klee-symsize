//===-- Memory.h ------------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_MEMORY_H
#define KLEE_MEMORY_H

#include "Context.h"
#include "TimingSolver.h"

#include "klee/Expr/Expr.h"

#include "llvm/ADT/StringExtras.h"

#include <string>
#include <vector>

namespace llvm {
  class Value;
}

namespace klee {

class ArrayCache;
class BitArray;
class ExecutionState;
class MemoryManager;
class Solver;

class MemoryObject {
  friend class STPBuilder;
  friend class ObjectState;
  friend class ExecutionState;
  friend class ref<MemoryObject>;
  friend class ref<const MemoryObject>;

private:
  static int counter;
  /// @brief Required by klee::ref-managed objects
  mutable class ReferenceCounter _refCount;

public:
  unsigned id;
  uint64_t address;

  /* TODO: add docs */
  ref<Expr> size;
  /* TODO: add docs */
  unsigned capacity;
  mutable std::string name;

  bool isLocal;
  mutable bool isGlobal;
  bool isFixed;

  bool isUserSpecified;

  MemoryManager *parent;

  /// "Location" for which this memory object was allocated. This
  /// should be either the allocating instruction or the global object
  /// it was allocated for (or whatever else makes sense).
  const llvm::Value *allocSite;
  
  /// A list of boolean expressions the user has requested be true of
  /// a counterexample. Mutable since we play a little fast and loose
  /// with allowing it to be added to during execution (although
  /// should sensibly be only at creation time).
  mutable std::vector< ref<Expr> > cexPreferences;

  bool canFree;

  // DO NOT IMPLEMENT
  MemoryObject(const MemoryObject &b);
  MemoryObject &operator=(const MemoryObject &b);

public:
  // XXX this is just a temp hack, should be removed
  explicit
  MemoryObject(uint64_t _address) 
    : id(counter++),
      address(_address),
      size(nullptr),
      capacity(0),
      isFixed(true),
      parent(NULL),
      allocSite(0),
      canFree(false) {
  }

  MemoryObject(uint64_t address, ref<Expr> size, unsigned capacity,
               bool isLocal, bool isGlobal, bool isFixed, bool canFree,
               const llvm::Value *allocSite,
               MemoryManager *parent)
    : id(counter++),
      address(address),
      size(size),
      capacity(capacity),
      name("unnamed"),
      isLocal(isLocal),
      isGlobal(isGlobal),
      isFixed(isFixed),
      isUserSpecified(false),
      parent(parent),
      allocSite(allocSite),
      canFree(canFree) {
    if (!hasFixedSize()) {
      /* this assumption is needed for pointer resolution */
      assert(capacity > 0);
    }
  }

  ~MemoryObject();

  /// Get an identifying string for this allocation.
  void getAllocInfo(std::string &result) const;

  void setName(std::string name) const {
    this->name = name;
  }

  ref<ConstantExpr> getBaseExpr() const { 
    return ConstantExpr::create(address, Context::get().getPointerWidth());
  }
  ref<Expr> getSizeExpr() const {
    return size;
    //return ConstantExpr::create(size, Context::get().getPointerWidth());
  }
  inline bool hasFixedSize() const {
    return isa<ConstantExpr>(size);
  }
  inline unsigned getFixedSize() const {
    ConstantExpr *c = dyn_cast<ConstantExpr>(size);
    if (c) {
      return c->getZExtValue();
    } else {
      assert(0);
    }
  }
  ref<Expr> getOffsetExpr(ref<Expr> pointer) const {
    return SubExpr::create(pointer, getBaseExpr());
  }
  ref<Expr> getBoundsCheckPointer(ref<Expr> pointer) const {
    return getBoundsCheckOffset(getOffsetExpr(pointer));
  }
  ref<Expr> getBoundsCheckPointer(ref<Expr> pointer, unsigned bytes) const {
    return getBoundsCheckOffset(getOffsetExpr(pointer), bytes);
  }

  ref<Expr> getBoundsCheckOffset(ref<Expr> offset) const {
    if (hasFixedSize() && getFixedSize() == 0) {
      return EqExpr::create(offset,
                            ConstantExpr::alloc(0, Context::get().getPointerWidth()));
    } else {
      return UltExpr::create(offset, getSizeExpr());
    }
  }
  ref<Expr> getBoundsCheckOffset(ref<Expr> offset, unsigned bytes) const {
    if (hasFixedSize()) {
      unsigned fixedSize = getFixedSize();
      if (bytes <= fixedSize) {
        return UltExpr::create(offset,
                               ConstantExpr::alloc(fixedSize - bytes + 1,
                                                   Context::get().getPointerWidth()));
      } else {
        return ConstantExpr::alloc(0, Expr::Bool);
      }
    }
    ref<Expr> bytesCheck = UleExpr::create(
      ConstantExpr::create(
        bytes,
        Context::get().getPointerWidth()
      ),
      size
    );
    ref<Expr> offsetCheck = UleExpr::create(
      offset,
      SubExpr::create(
        size,
        ConstantExpr::create(bytes, Context::get().getPointerWidth())
      )
    );
    return AndExpr::create(bytesCheck, offsetCheck);
  }

  /// Compare this object with memory object b.
  /// \param b memory object to compare with
  /// \return <0 if this is smaller, 0 if both are equal, >0 if b is smaller
  int compare(const MemoryObject &b) const {
    // Short-cut with id
    if (id == b.id)
      return 0;
    if (address != b.address)
      return (address < b.address ? -1 : 1);

    assert(hasFixedSize() && b.hasFixedSize());
    if (getFixedSize() != b.getFixedSize())
      return (getFixedSize() < b.getFixedSize() ? -1 : 1);
    //if (size != b.size)
    //  return (size < b.size ? -1 : 1);

    if (allocSite != b.allocSite)
      return (allocSite < b.allocSite ? -1 : 1);

    return 0;
  }
};

class ObjectState {
private:
  friend class AddressSpace;
  friend class ref<ObjectState>;

  unsigned copyOnWriteOwner; // exclusively for AddressSpace

  /// @brief Required by klee::ref-managed objects
  class ReferenceCounter _refCount;

  ref<const MemoryObject> object;

  uint8_t *concreteStore;

  // XXX cleanup name of flushMask (its backwards or something)
  BitArray *concreteMask;

  // mutable because may need flushed during read of const
  mutable BitArray *flushMask;

  ref<Expr> *knownSymbolics;

  // mutable because we may need flush during read of const
  mutable UpdateList updates;

  /* TODO: this is not a bound, rename */
  mutable unsigned actualBound;

  mutable unsigned upperBound;

public:
  unsigned size;

  bool readOnly;

public:
  /// Create a new object state for the given memory object with concrete
  /// contents. The initial contents are undefined, it is the callers
  /// responsibility to initialize the object contents appropriately.
  ObjectState(const MemoryObject *mo);

  /// Create a new object state for the given memory object with symbolic
  /// contents.
  ObjectState(const MemoryObject *mo, const Array *array);

  ObjectState(const ObjectState &os);
  ~ObjectState();

  const MemoryObject *getObject() const { return object.get(); }

  void setReadOnly(bool ro) { readOnly = ro; }

  // make contents all concrete and zero
  void initializeToZero();
  // make contents all concrete and random
  void initializeToRandom();

  ref<Expr> read(ref<Expr> offset, Expr::Width width, bool track = true) const;
  ref<Expr> read(unsigned offset, Expr::Width width, bool track = true) const;
  ref<Expr> read8(unsigned offset, bool track = true) const;

  // return bytes written.
  void write(unsigned offset, ref<Expr> value);
  void write(ref<Expr> offset, ref<Expr> value);

  void write8(unsigned offset, uint8_t value);
  void write16(unsigned offset, uint16_t value);
  void write32(unsigned offset, uint32_t value);
  void write64(unsigned offset, uint64_t value);
  void print() const;

  /*
    Looks at all the symbolic bytes of this object, gets a value for them
    from the solver and puts them in the concreteStore.
  */
  void flushToConcreteStore(TimingSolver *solver,
                            const ExecutionState &state) const;

  unsigned getActualBound() const {
    return actualBound;
  }

  void setActualBound(unsigned bound) const {
    /* TODO: validate that there were no symbolic-offset writes */
    actualBound = bound;
  }

  unsigned getUpperBound() const {
    return upperBound;
  }

  void setUpperBound(unsigned bound) {
    if (bound < upperBound) {
      upperBound = bound;
    }
  }

  void resetUpperBound(unsigned bound) const {
    upperBound = bound;
  }

private:
  const UpdateList &getUpdates() const;

  void makeConcrete();

  void makeSymbolic();

  ref<Expr> read8(ref<Expr> offset, bool track) const;
  void write8(unsigned offset, ref<Expr> value);
  void write8(ref<Expr> offset, ref<Expr> value);

  void fastRangeCheckOffset(ref<Expr> offset, unsigned *base_r, 
                            unsigned *size_r) const;
  void flushRangeForRead(unsigned rangeBase, unsigned rangeSize) const;
  void flushRangeForWrite(unsigned rangeBase, unsigned rangeSize);

  bool isByteConcrete(unsigned offset) const;
  bool isByteFlushed(unsigned offset) const;
  bool isByteKnownSymbolic(unsigned offset) const;

  void markByteConcrete(unsigned offset);
  void markByteSymbolic(unsigned offset);
  void markByteFlushed(unsigned offset);
  void markByteUnflushed(unsigned offset);
  void setKnownSymbolic(unsigned offset, Expr *value);

  ArrayCache *getArrayCache() const;

  void onConcreteAccess(unsigned offset, bool track = true) const;
  void onSymbolicAccess(ref<Expr> offset, bool track = true) const;
};
  
} // End klee namespace

#endif /* KLEE_MEMORY_H */
