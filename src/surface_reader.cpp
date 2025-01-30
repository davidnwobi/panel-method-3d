#include "surface/surface_reader.hpp"
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"
#include <filesystem>
#include <string>
#include <utility>

template int stringToNum<int>(const std::string &str);
template double stringToNum<double>(const std::string &str);

template void readTable<SurfacePanel::NodeMatrix>(SurfaceFileReader &,
                                                  SurfacePanel::NodeMatrix &);
template void
readTable<SurfacePanel::FaceNodeIdxMatrix>(SurfaceFileReader &,
                                           SurfacePanel::FaceNodeIdxMatrix &);
template void
readTable<WakePanel::TrailingEdgeIdxMatrix>(SurfaceFileReader &,
                                            WakePanel::TrailingEdgeIdxMatrix &);

int readDimensions(SurfaceFileReader &fileReader) {
  // Skips a header line and gets the integer under it
  fileReader.getNextLine();
  return std::stoi(fileReader.getNextLine()[0]);
}

SurfacePanel readSurface(SurfaceFileReader &fileReader) {
  std::size_t noSurfNode = readDimensions(fileReader);
  std::size_t noFaces = readDimensions(fileReader);
  std::size_t nXsecs = readDimensions(fileReader);
  std::size_t nYsecs = readDimensions(fileReader);

  SurfacePanel::NodeMatrix nodeMatrix(
      noSurfNode, SurfacePanel::NodeMatrix::ColsAtCompileTime);
  readTable(fileReader, nodeMatrix);

  SurfacePanel::FaceNodeIdxMatrix faceNodeIdx(
      noFaces, SurfacePanel::FaceNodeIdxMatrix::ColsAtCompileTime);
  readTable(fileReader, faceNodeIdx);

  return SurfacePanel(std::move(nodeMatrix), std::move(faceNodeIdx), nXsecs,
                      nYsecs);
}

WakePanel readWake(SurfaceFileReader &fileReader) {
  WakePanel wake(readSurface(fileReader));
  auto noWakePanels = wake.mFaceNodeIdx.rows();

  WakePanel::TrailingEdgeIdxMatrix trailingEdgeIdx(
      noWakePanels, WakePanel::TrailingEdgeIdxMatrix::ColsAtCompileTime);
  readTable(fileReader, trailingEdgeIdx);

  wake.mTrailingEdgeIdx = std::move(trailingEdgeIdx);
  return wake;
}

std::vector<PanelSet>
readConvertedComponentsFromFile(const std::filesystem::path &filePath,
                                char delim) {
  SurfaceFileReader fileReader(filePath, delim);

  // Skip headers
  fileReader.getNextLine();
  int numberOfSurfaces = std::stoi(fileReader.getNextLine()[0]);

  std::vector<PanelSet> psets;
  for (int iSurface = 0; iSurface < numberOfSurfaces; iSurface++) {
    // Skip component name
    PanelSet pset;
    fileReader.getNextLine();
    pset.body = readSurface(fileReader);
    fileReader.getNextLine();
    pset.wake = readWake(fileReader);

    psets.emplace_back(std::move(pset));
  }

  return psets;
}

std::vector<std::string> SurfaceFileReader::splitLine(std::string &data) {
  std::istringstream inputBuffer(data);
  std::string token;
  std::vector<std::string> tokens;
  while (std::getline(inputBuffer, token, mDelim)) {
    tokens.emplace_back(std::move(token));
  }
  return tokens;
}
SurfaceFileReader::SurfaceFileReader(const std::filesystem::path &filePath,
                                     char delim)
    : mSurfaceFile(filePath), mDelim(delim) {
  if (!mSurfaceFile.is_open()) {
    std::cerr << "Unable to open file at "
              << std::filesystem::absolute(filePath);
  }
}
std::vector<std::string> SurfaceFileReader::getNextLine() {
  // Read line
  std::string data;
  std::getline(mSurfaceFile, data);
  return splitLine(data);
}
[[nodiscard]] bool SurfaceFileReader::isEOF() { return mSurfaceFile.eof(); }
