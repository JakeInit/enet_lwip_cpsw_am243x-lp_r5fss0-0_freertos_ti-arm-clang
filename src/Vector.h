/*
 * Vector.h
 *
 *  Created on: Mar 21, 2022
 *      Author: marekmosna
 *      link: https://github.com/eteran/c-vector/blob/master/cvector.h
 */

#ifndef SRC_VECTOR_H_
#define SRC_VECTOR_H_

#include <assert.h> /* for assert */
#include <stdlib.h> /* for malloc/realloc/free */
#include <string.h> /* for memcpy/memmove */

/* Vector heap implemented using C library malloc() */

/**
 * @brief Vector - The vector type used in this library
 */
#define Vector(type) type*

/**
 * @brief vectorCapacity - gets the current capacity of the vector
 * @param vec - the vector
 * @return the capacity as a size_t
 */
#define vectorCapacity(vec) ((vec) ? ((size_t *)(vec))[-1] : (size_t)0)

/**
 * @brief vectorSize - gets the current size of the vector
 * @param vec - the vector
 * @return the size as a size_t
 */
#define vectorSize(vec) ((vec) ? ((size_t *)(vec))[-2] : (size_t)0)

/**
 * @brief vectorEmpty - returns non-zero if the vector is empty
 * @param vec - the vector
 * @return non-zero if empty, zero if non-empty
 */
#define vectorEmpty(vec) (vectorSize(vec) == 0)

/**
 * @brief vectorReserve - Requests that the vector capacity be at least enough
 * to contain n elements. If n is greater than the current vector capacity, the
 * function causes the container to reallocate its storage increasing its
 * capacity to n (or greater).
 * @param vec - the vector
 * @param n - Minimum capacity for the vector.
 * @return void
 */
#define vectorReserve(vec, capacity) \
    do { \
        size_t cv_cap = vectorCapacity(vec); \
        if (cv_cap < (capacity)) \
        { \
            vectorGrow((vec), (capacity));   \
        }                                      \
    } while (0)

/**
 * @brief vectorErase - removes the element at index i from the vector
 * @param vec - the vector
 * @param i - index of element to remove
 * @return void
 */
#define vectorErase(vec, i) \
    do { \
        if ((vec)) \
        { \
            const size_t cv_sz = vectorSize(vec); \
            if ((i) < cv_sz) \
            { \
                vectorSetSize((vec), cv_sz - 1); \
                memmove((vec) + (i), (vec) + (i) + 1, sizeof(*(vec)) * (cv_sz - 1 - (i))); \
            } \
        } \
    } while (0)

/**
 * @brief vectorFree - frees all memory associated with the vector
 * @param vec - the vector
 * @return void
 */
#define vectorFree(vec) \
    do { \
        if ((vec)) \
        { \
            size_t *p1 = &((size_t *)(vec))[-2]; \
            free(p1); \
        } \
    } while (0)

/**
 * @brief vectorBegin - returns an iterator to first element of the vector
 * @param vec - the vector
 * @return a pointer to the first element (or NULL)
 */
#define vectorBegin(vec) (vec)

/**
 * @brief vectorEnd - returns an iterator to one past the last element of the vector
 * @param vec - the vector
 * @return a pointer to one past the last element (or NULL)
 */
#define vectorEnd(vec) ((vec) ? &((vec)[vectorSize(vec)]) : NULL)

/**
 * @brief vectorComputeNextGrowLog - returns the computed size in next vector grow
 * size is increased by multiplication of 2
 * @param size - current size
 * @return size after next vector grow
 */
#define vectorComputeNextGrowLog(size) ((size) ? ((size) << 1) : 1)

/**
 * @brief vectorComputeNextGrow - returns the computed size in next vector grow
 * size is increased by 1
 * @param size - current size
 * @return size after next vector grow
 */
#define vectorComputeNextGrow(size) ((size) + 1)

/**
 * @brief vectorPushBack - adds an element to the end of the vector
 * @param vec - the vector
 * @param value - the value to add
 * @return void
 */
#define vectorPushBack(vec, value) \
    do { \
        size_t cv_cap = vectorCapacity(vec); \
        if (cv_cap <= vectorSize(vec)) \
        { \
            vectorGrow((vec), vectorComputeNextGrow(cv_cap)); \
        } \
        vec[vectorSize(vec)] = (value); \
        vectorSetSize((vec), vectorSize(vec) + 1); \
    } while (0)

/**
 * @brief vectorInsert - insert element at position pos to the vector
 * @param vec - the vector
 * @param pos - position in the vector where the new elements are inserted.
 * @param val - value to be copied (or moved) to the inserted elements.
 * @return void
 */
#define vectorInsert(vec, pos, val) \
    do { \
        if (vectorCapacity(vec) <= vectorSize(vec) + 1) \
        { \
            vectorGrow((vec), vectorComputeNextGrowvectorComputeNextGrow(vectorCapacity((vec)))); \
        } \
        if (pos < vectorSize(vec)) \
        { \
            memmove((vec) + (pos) + 1, (vec) + (pos), sizeof(*(vec)) * ((vectorSize(vec) + 1) - (pos))); \
        } \
        (vec)[(pos)] = (val); \
        vectorSetSize((vec), vectorSize(vec) + 1); \
    } while (0)

/**
 * @brief vectorPopBack - removes the last element from the vector
 * @param vec - the vector
 * @return void
 */
#define vectorPopBack(vec) \
    do { \
        vectorSetSize((vec), vectorSize(vec) - 1); \
    } while (0)

/**
 * @brief vectorCopy - copy a vector
 * @param from - the original vector
 * @param to - destination to which the function copy to
 * @return void
 */
#define vectorCopy(from, to) \
    do { \
        if ((from)) \
        { \
            vectorGrow(to, vectorSize(from)); \
            vectorSetSize(to, vectorSize(from)); \
            memcpy((to), (from), vectorSize(from) * sizeof(*(from))); \
        } \
    } while (0)

/**
 * @brief vectorSetCapacity - For internal use, sets the capacity variable of the vector
 * @param vec - the vector
 * @param size - the new capacity to set
 * @return void
 */
#define vectorSetCapacity(vec, size) \
    do { \
        if ((vec)) \
        { \
            ((size_t *)(vec))[-1] = (size); \
        } \
    } while (0)

/**
 * @brief vectorSetSize - For internal use, sets the size variable of the vector
 * @param vec - the vector
 * @param size - the new capacity to set
 * @return void
 */
#define vectorSetSize(vec, size) \
    do { \
        if ((vec)) \
        { \
            ((size_t *)(vec))[-2] = (size); \
        } \
    } while (0)

/**
 * @brief vectorGrow - For internal use, ensures that the vector is at least <count> elements big
 * @param vec - the vector
 * @param count - the new capacity to set
 * @return void
 */
#define vectorGrow(vec, count) \
    do { \
        const size_t cv_sz = (count) * sizeof(*(vec)) + (sizeof(size_t) * 2); \
        if ((vec)) \
        { \
            size_t *cv_p1 = &((size_t *)(vec))[-2]; \
            size_t *cv_p2 = realloc(cv_p1, (cv_sz)); \
            assert(cv_p2); \
            (vec) = (void *)(&cv_p2[2]); \
            vectorSetCapacity((vec), (count)); \
        } \
        else \
        { \
            size_t *cv_p = malloc(cv_sz); \
            assert(cv_p); \
            (vec) = (void *)(&cv_p[2]); \
            vectorSetCapacity((vec), (count)); \
            vectorSetSize((vec), 0); \
        } \
    } while (0)

#endif /* SRC_VECTOR_H_ */
