/**
 * @file utils.hpp
 * @brief Defines utility functions
 *
 */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cxxabi.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#define UNUSED(obj) (void)(obj);
template <typename... Args> void print(Args &&...args) {
  (std::cout << ... << args) << "\n";
}

template <typename T>
concept Printable = requires(std::ostream &os, const T &a) {
  { os << a } -> std::same_as<std::ostream &>;
};

template <std::ranges::input_range R> void print_range(const R &inputRange) {
  using T = std::decay_t<decltype(*inputRange.begin())>;
  static_assert(Printable<T>);
  std::ranges::copy(inputRange, std::ostream_iterator<T>(std::cout, " \n"));
  std::cout << "\n";
}

#define PRINT_RANGE(container) print_range(container);
#define assertm(exp, msg) assert(((void)msg, exp))
std::vector<std::string> split(const std::string &line,
                               const std::string &delimiter);
/**
 * @class FileReader
 * @brief Abstract base class for file readers.
 *
 * The FileReader class serves as an abstract base class that defines the
 * interface for reading and saving data from/to files. Subclasses must
 * implement the pure virtual functions read_data() and save_data().
 */
class FileReader {
public:
  /**
   * @brief Reads data from a file.
   *
   * Subclasses should provide specific implementation for reading data from the
   * given file.
   *
   * @param fileToOpen The name of the file to open and read data from.
   * @return Eigen::MatrixXd The data read from the file.
   */
  virtual Eigen::MatrixXd read_data(std::string fileToOpen) = 0;

  /**
   * @brief Saves data to a file.
   *
   * Subclasses should provide specific implementation for saving data to the
   * given file.
   *
   * @param fileName The name of the file to save data to.
   * @param matrix The matrix data to save.
   */
  virtual void save_data(std::string fileName, Eigen::MatrixXd matrix) = 0;

  /**
   * @brief Virtual destructor.
   *
   * Ensures proper cleanup of derived class objects.
   */
  virtual ~FileReader() = default;
};

/**
 * @class CSVFileReader
 * @brief Concrete implementation of the FileReader class for CSV files.
 *
 * The CSVFileReader class provides an implementation of the FileReader
 * interface for reading from and writing to CSV files. It supports custom
 * delimiters and optionally skipping headers.
 */
class CSVFileReader : public FileReader {
public:
  /**
   * @brief Constructs a CSVFileReader object.
   *
   * Initializes the CSV file reader with a specified delimiter and header
   * option.
   *
   * @param delimiter The delimiter used in the CSV file.
   * @param has_header Indicates if the CSV file contains a header row.
   */
  explicit CSVFileReader(std::string delimiter, bool has_header);

  /**
   * @brief Reads data from a CSV file.
   *
   * Implements reading data from the specified CSV file.
   *
   * @param fileToOpen The name of the CSV file to open and read data from.
   * @return Eigen::MatrixXd The data read from the CSV file.
   */
  Eigen::MatrixXd read_data(std::string fileToOpen) override;

  /**
   * @brief Saves data to a CSV file.
   *
   * Implements saving data to the specified CSV file.
   *
   * @param fileName The name of the CSV file to save data to.
   * @param matrix The matrix data to save.
   */
  void save_data(std::string fileName, Eigen::MatrixXd matrix) override;

  /**
   * @brief Virtual destructor.
   *
   * Ensures proper cleanup of CSVFileReader objects.
   */
  ~CSVFileReader() = default;

private:
  const std::string delimiter; /**< The delimiter used in the CSV file. */
  bool has_header; /**< Indicates if the CSV file contains a header row. */
};

/**
 * @class FileReaderFactory
 * @brief Factory class for creating FileReader objects.
 *
 * The FileReaderFactory class provides a static method for creating
 * FileReader objects based on the specified file type.
 */
class FileReaderFactory {
public:
  /**
   * @brief Creates a FileReader object based on the file type.
   *
   * @param file_type The type of the file (e.g., "csv").
   * @param delimiter The delimiter used in the file (default is ",").
   * @param has_header Indicates if the file contains a header row (default is
   * false).
   * @return std::unique_ptr<FileReader> A unique pointer to the created
   * FileReader object.
   */
  static std::unique_ptr<FileReader>
  make_file_reader(std::string file_type, std::string delimiter = ",",
                   bool has_header = false) {
    if (file_type == "csv" || file_type == "dat") {
      return std::make_unique<CSVFileReader>(delimiter, has_header);
    } else {
      throw std::invalid_argument("Unsupported file type: " + file_type);
    }
  }
};

/**
 * @brief Performs an accumulation operation over a range of elements, moving
 * the accumulator in each step.
 *
 * @tparam InputIt The type of the input iterator.
 * @tparam T The type of the accumulator.
 * @tparam BinaryOp The type of the binary operation to apply.
 *
 * @param first Iterator pointing to the start of the range.
 * @param last Iterator pointing to the end of the range.
 * @param init Initial value of the accumulator.
 * @param op Binary operation to apply, taking the current accumulator and an
 * element as arguments.
 *
 * @return The result of accumulating the range with the specified operation,
 * using move semantics for the accumulator.
 *
 * @note This function is useful for operations where the accumulator can be
 * moved for performance optimization. It is intended for use with
 * `std::make_move_iterator`
 */
template <class InputIt, class T, class BinaryOp>
T accumulate_move(InputIt first, InputIt last, T init, BinaryOp op) {
  for (; first != last; ++first) {
    init = op(std::move(init), *first);
  }
  return init;
}

/**
 * @brief Creates and initializes a container by transforming a range of
 * elements with a unary operation.
 *
 * @tparam Container The type of the container to initialize (e.g.,
 * std::vector).
 * @tparam UnaryOp The type of the unary operation to apply.
 * @tparam InputIt The type of the input iterator.
 *
 * @param first Iterator pointing to the start of the range.
 * @param last Iterator pointing to the end of the range.
 * @param op Unary operation to apply to each element in the range.
 *
 * @return A container filled with the results of applying the unary operation
 * to the range.
 *
 * @note The container's size is reserved in advance for efficiency. Use this
 * for transforming elements into a new container.
 */
template <class Container, class UnaryOp, class InputIt>
Container initialize_transform(InputIt first, InputIt last, UnaryOp op) {
  auto size = std::distance(first, last);

  Container container;
  container.reserve(size);
  std::transform(first, last, std::back_inserter(container), op);

  return container;
}

/**
 * @brief Creates and initializes a container by transforming two ranges of
 * elements with a binary operation.
 *
 * @tparam Container The type of the container to initialize (e.g.,
 * std::vector).
 * @tparam BinaryOp The type of the binary operation to apply.
 * @tparam ForwardIt1 The type of the first input iterator.
 * @tparam ForwardIt2 The type of the second input iterator.
 *
 * @param first1 Iterator pointing to the start of the first range.
 * @param last1 Iterator pointing to the end of the first range.
 * @param first2 Iterator pointing to the start of the second range.
 * @param op Binary operation to apply, taking one element from each range as
 * arguments.
 *
 * @return A container filled with the results of applying the binary operation
 * to pairs of elements from the two ranges.
 *
 * @note The container's size is reserved in advance for efficiency. This
 * function assumes both ranges are of the same size.
 */
template <class Container, class BinaryOp, class ForwardIt1, class ForwardIt2>
Container initialize_transform(ForwardIt1 first1, ForwardIt1 last1,
                               ForwardIt2 first2, BinaryOp op) {
  auto size = std::distance(first1, last1);

  Container container;
  container.reserve(size);
  std::transform(first1, last1, first2, std::back_inserter(container), op);

  return container;
}

template <class EigenMatrixTypeA, class EigenMatrixTypeB>
constexpr auto vMerge(EigenMatrixTypeA &&matA, EigenMatrixTypeB &&matB) {

  // Figure out whether we can reuse memory from here rather than making copies.
  // Actually we can. Get PanelGeometry to consume this and have a decompose
  // method.

  // Vectically merge two eigen matrices into one. Consuming!
  using AType = std::decay_t<EigenMatrixTypeA>;
  assertm(matA.cols() == matB.cols(),
          "MAT_A_AND_MAT_B_MUST_HAVE_THE_SAME_NUMBER_OF_COLUMNS");

  AType matC(matA.rows() + matB.rows(), matA.cols());
  matC << std::forward<EigenMatrixTypeA>(matA),
      std::forward<EigenMatrixTypeB>(matB);

  return matC;
}

template <class EigenMatrixType>
EigenMatrixType hMerge(EigenMatrixType matA, EigenMatrixType matB) {

  // Figure out whether we can reuse memory from here rather than making copies.
  // Actually we can. Get PanelGeometry to consume this and have a decompose
  // method.

  // Vectically merge two eigen matrices into one. Consuming!
  assertm(matA.rows() == matB.rows(),
          "MAT_A_AND_MAT_B_MUST_HAVE_THE_SAME_NUMBER_OF_ROWS");

  EigenMatrixType matC(matA.rows(), matA.cols() + matB.cols());
  auto a = matA.cols();
  auto b = matB.cols();

  matC.middleCols(0, a) = std::move(matA);
  matC.middleCols(a, b) = std::move(matB);

  return matC;
}

template <class ItT, class ItU, class Func>
void apply_adjacent(ItT begin, ItT end, ItU begin2, Func func) {
  using U = typename std::iterator_traits<ItU>::value_type;
  static_assert(std::is_convertible_v<decltype(func(*begin, *(begin + 1))), U>,
                "Function return cannot be stored in Output container");

  int N = std::distance(begin, end);
  for (int i = 0; i < N - 1; i++) {
    *begin2 = func(*begin, *(begin + 1));
    begin++, begin2++;
  }
}

template <class ItT, class ItU, class Func>
void apply_adjacent_circular(ItT begin, ItT end, ItU begin2, Func func) {

  apply_adjacent(begin, end, begin2, func);
  auto rangeEnd = std::distance(begin, end - 1);
  *(begin2 + rangeEnd) = func(*(end - 1), *begin);
}

template <typename T> void print_vector(const std::vector<T> &vec) {
  for (const T &elem : vec) {
    std::cout << elem << " ";
  }
  std::cout << "\n";
}

template <class EigenMatType, std::integral T>
std::vector<EigenMatType> hChunk(EigenMatType combined,
                                 const std::vector<T> &chunkIdx) {

  T prev = 0;
  return initialize_transform<std::vector<EigenMatType>>(
      chunkIdx.begin(), chunkIdx.end(), [&](T iChunk) -> EigenMatType {
        return combined.middleRows(prev, iChunk);
        prev += iChunk;
      });
}

std::string demangle(const char *mangledName);
#define PRINT_TYPE(obj)                                                        \
  std::cout << "Type of " #obj ": " << demangle(typeid(obj).name())            \
            << std::endl;

Eigen::ArrayXd
rowwiseDotProduct(const Eigen::ArrayXXd &a1,
                  const Eigen::Array<double, 1, -1, Eigen::RowMajor> &a2);

template <typename MatA, typename MatB, typename... Rest>
auto vMergeRecursive(MatA &&matA, MatB &&matB,
                     Rest... mats) -> std::decay_t<MatA> {
  auto merged = vMerge(std::forward<MatA>(matA), std::forward<MatB>(matB));

  if constexpr (sizeof...(mats) == 0) {
    return merged;
  } else {
    return vMergeRecursive(std::move(merged), std::forward<Rest>(mats)...);
  }
}
