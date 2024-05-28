#include "control_lib/UsefullFunctions.h"

void printArray(double* theArray, int numberOfRows, int numberOfColumns)
{
  for (int x = 0; x < numberOfRows; x++)
  {
    for (int y = 0; y < numberOfColumns; y++)
    {
      std::cout << theArray[x * numberOfColumns + y] << " ";
    }
    std::cout << std::endl;
  }
}

template <typename containerType>
void printContainer(containerType& container)
{
  std::cout << std::endl << "CONTAINER_DATA:\n{";
  int i = 0;
  for (const auto& value : container)
  {
    if (i == container.size() - 1)
    {
      std::cout << value << "}\n";
    }
    else
    {
      std::cout << value << ", ";
    }
    i++;
  }
}

void RadianToDegree(std::vector<double> radVector)
{
  const double pi = 3.1415926535897;
  std::vector<double> degVector;
  int vecLength = radVector.size();

  for (int i = 0; i < vecLength; i++)
  {
    degVector.push_back(radVector[i] * (180 / pi));
  }

  printContainer(degVector);
}

void write_csv(std::string filename,
               std::vector<std::pair<std::string, std::vector<double> > > dataset)
{
  // Make a CSV file with one or more columns of integer values
  // Each column of data is represented by the pair <column name, column data>
  //   as std::pair<std::string, std::vector<int>>
  // The dataset is represented as a vector of these columns
  // Note that all columns should be the same size

  // Create an output filestream object
  std::ofstream myFile(filename);

  // Send column names to the stream
  for (int j = 0; j < dataset.size(); ++j)
  {
    myFile << dataset.at(j).first;
    if (j != dataset.size() - 1)
      myFile << ","; // No comma at end of line
  }
  myFile << "\n";

  // Send data to the stream
  for (int i = 0; i < dataset.at(0).second.size(); ++i)
  {
    for (int j = 0; j < dataset.size(); ++j)
    {
      myFile << dataset.at(j).second.at(i);
      if (j != dataset.size() - 1)
        myFile << ","; // No comma at end of line
    }
    myFile << "\n";
  }

  // Close the file
  myFile.close();


  /*// Make three vectors, each of length 100 filled with 1s, 2s, and 3s
  std::vector<int> vec1(100, 1);
  std::vector<int> vec2(100, 2);
  std::vector<int> vec3(100, 3);

  // Wrap into a vector
  std::vector<std::pair<std::string, std::vector<int>>> vals = {{"One", vec1}, {"Two", vec2},
  {"Three", vec3}};

  // Write the vector to CSV
  write_csv("three_cols.csv", vals);*/
}

void wait(unsigned int milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
