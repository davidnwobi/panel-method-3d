#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <iterator>
#include <fstream>
#include <iterator>
#include <type_traits>
#include <Eigen/Core>
#include "utils/utils.hpp"

std::vector<std::string> split(const std::string &line, const std::string &delimiter)
{
    std::vector<std::string> result;
    std::string token;
    std::istringstream tokenStream(line);
    char c;

    // Process each character in the input line
    while (tokenStream.get(c))
    {
        // Check if the current character is a delimiter or space
        if (std::string{c} == std::string{" "} || delimiter == std::string{c})
        {
            if (!token.empty())
            {
                // If token is not empty, add it to the result and clear token
                result.push_back(token);
                token.clear();
            }
        }
        else
        {
            // Append character to token
            token += c;
        }
    }

    // Add the last token to the result if not empty
    if (!token.empty())
    {
        result.push_back(token);
    }

    return result;
}

/**
 * @brief Constructs a CSVFileReader object.
 *
 * Initializes the CSV file reader with a specified delimiter and header option.
 *
 * @param delimiter The delimiter used in the CSV file.
 * @param has_header Indicates if the CSV file contains a header row.
 */
CSVFileReader::CSVFileReader(std::string delimiter, bool has_header) : delimiter(delimiter), has_header(has_header) {}

/**
 * @brief Reads data from a CSV file.
 *
 * Implements reading data from the specified CSV file.
 *
 * @param fileToOpen The name of the CSV file to open and read data from.
 * @return Eigen::MatrixXd The data read from the CSV file.
 */
Eigen::MatrixXd CSVFileReader::read_data(std::string fileToOpen)
{
    // Vector to store matrix entries row-wise
    std::vector<double> matrixEntries;
    // Input file stream to read the CSV file
    std::ifstream matrixDataFile(fileToOpen);
    if (!matrixDataFile.is_open())
    {
        std::cerr << "Error reading from file: " << fileToOpen << std::endl;
        exit(1);
    }

    std::string matrixRowString;
    int matrixRowNumber = 0;

    // Skip the header row if present
    if (has_header)
    {
        getline(matrixDataFile, matrixRowString);
    }
    // Read the file line by line
    while (getline(matrixDataFile, matrixRowString))
    {
        // Split the row into entries based on the delimiter
        std::vector<std::string> nums = split(matrixRowString, delimiter);
        std::transform(nums.begin(), nums.end(), std::back_inserter(matrixEntries), [](const std::string &s)
                       { return std::stod(s); });
        matrixRowNumber++;
    }

    // Map the vector to Eigen matrix format
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        matrixEntries.data(),
        matrixRowNumber,
        matrixEntries.size() / matrixRowNumber);
}

/**
 * @brief Saves data to a CSV file.
 *
 * Implements saving data to the specified CSV file.
 *
 * @param fileName The name of the CSV file to save data to.
 * @param matrix The matrix data to save.
 */
void CSVFileReader::save_data(std::string fileName, Eigen::MatrixXd matrix)
{
    const std::string delim{delimiter};
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, delim, "\n");

    // Output file stream to write the CSV file
    std::ofstream file(fileName);
    if (file.is_open())
    {
        // Write the matrix data to the file using the specified format
        file << matrix.format(CSVFormat);
        file.close();
    }
}



std::string demangle(const char* mangledName) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(mangledName, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled != nullptr) ? demangled : mangledName;
    free(demangled);
    return result;
}


Eigen::ArrayXd rowwiseDotProduct(const Eigen::ArrayXXd& a1, const Eigen::Array<double, 1, -1, Eigen::RowMajor>& a2){
	return (a1.rowwise() * a2).rowwise().sum();
}
