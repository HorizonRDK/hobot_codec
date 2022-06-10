// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDE_ARRQUEUE_HPP_
#define INCLUDE_ARRQUEUE_HPP_
#include <iostream>

// queue implemented as an array
typedef void (*fnDelItem)(void *pItem);
typedef void (*fnDeepInitItem)(void *pItem, void *pSrc);

template<class T, int size = 100>
class CArrayQueue {
 public:
  explicit CArrayQueue(fnDelItem fnDel = nullptr) {
    first = last = -1;
    mFnDelItem = fnDel;
    memset(storage, 0, size*sizeof(T));
  }
  ~CArrayQueue() {
    for (int nIdx = 0; nIdx < size; ++nIdx) {
      if (mFnDelItem)
        mFnDelItem(reinterpret_cast<void*>(&storage[nIdx]));
    }
  }
  void enqueue(T item, fnDeepInitItem fnInit);
  T* dequeue();
  T* dequeue_val();
  bool isFull() {
    return first == 0 && last == size-1 || first == last + 1;
  }
  bool isEmpty() {
    return first == -1;
  }

 private:
  std::mutex          m_MtxArr;
  fnDelItem mFnDelItem;
  int first;  // point to the first element that can be take away from the queue.
  int last;   // point to the last element that was put into the queue recently.
  // Do not use variable-length arrays. Use an appropriately named ('k' followed by CamelCase)
  // compile-time constant for the size.
  // T storage[size];
  T storage[30];
};
template<class T, int size>
void CArrayQueue<T, size>::enqueue(T el, fnDeepInitItem fnInit) {
  m_MtxArr.lock();
  if (!isFull()) {
    int nCurItem = 0;
    if (last == size-1 || last == -1) {
      // storage[0] = el;
      last = 0;
      if (first == -1)
        first = 0;
    } else {
      // storage[++last] = el;
      nCurItem = ++last;
    }
    // printf("[enqueue]->cur=%d,first=%d,last=%d,obj=0x%x.\n", nCurItem, first, last, &storage[nCurItem]);
    if (fnInit)
      fnInit(&storage[nCurItem], &el);
  }
  // else cout << "Full queue.\n";
  m_MtxArr.unlock();
}
template<class T, int size>
T* CArrayQueue<T, size>::dequeue_val() {
  T *tmp = nullptr;
  m_MtxArr.lock();
  if (!isEmpty()) {
    tmp = &storage[first];
  }
  m_MtxArr.unlock();
  // printf("[dequeue-val]->first=%d,last=%d,obj=0x%x.\n", first, last, tmp);
  return tmp;
}

template<class T, int size>
T* CArrayQueue<T, size>::dequeue() {
  T *tmp = nullptr;
  m_MtxArr.lock();
  if (!isEmpty()) {
    tmp = &storage[first];
    if (first == last)
      last = first = -1;
    else if (first == size-1)
      first = 0;
    else
      first++;
  }
  m_MtxArr.unlock();
  // printf("[dequeue]->first=%d,last=%d,obj=0x%x.\n", first, last, tmp);
  return tmp;
}

#endif  // INCLUDE_ARRQUEUE_HPP_
