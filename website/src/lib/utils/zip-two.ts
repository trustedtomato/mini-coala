export function* zipTwo<T, U>(a: Iterable<T>, b: Iterable<U>): Generator<[T, U]> {
  const aIterator = a[Symbol.iterator]()
  const bIterator = b[Symbol.iterator]()
  while (true) {
    const aResult = aIterator.next()
    const bResult = bIterator.next()
    if (aResult.done || bResult.done) {
      break
    }
    yield [aResult.value, bResult.value]
  }
}
