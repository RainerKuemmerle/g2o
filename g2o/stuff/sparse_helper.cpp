#include "sparse_helper.h"

#include <string>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace std;

namespace g2o {

  namespace {
    struct TripletEntry
    {
      int r, c;
      double x;
      TripletEntry(int r_, int c_, double x_) : r(r_), c(c_), x(x_) {}
    };
    struct TripletColSort
    {
      bool operator()(const TripletEntry& e1, const TripletEntry& e2) const
      {
        return e1.c < e2.c || (e1.c == e2.c && e1.r < e2.r);
      }
    };
  }

  bool writeVector(const char* filename, const double*v, int n)
  {
    ofstream os(filename);
    os << fixed;
    for (int i=0; i<n; i++)
      os << *v++ << endl;
    return os.good();
  }

  bool writeCCSMatrix(const char* filename, int rows, int cols, const int* Ap, const int* Ai, const double* Ax, bool upperTriangleSymmetric)
  {
    vector<TripletEntry> entries;
    entries.reserve(Ap[cols]);
    for (int i=0; i < cols; i++) {
      const int& rbeg = Ap[i];
      const int& rend = Ap[i+1];
      for (int j = rbeg; j < rend; j++) {
        entries.push_back(TripletEntry(Ai[j], i, Ax[j]));
        if (upperTriangleSymmetric && Ai[j] != i)
          entries.push_back(TripletEntry(i, Ai[j], Ax[j]));
      }
    }
    sort(entries.begin(), entries.end(), TripletColSort());

    string name = filename;
    std::string::size_type lastDot = name.find_last_of('.');
    if (lastDot != std::string::npos) 
      name = name.substr(0, lastDot);

    std::ofstream fout(filename);
    fout << "# name: " << name << std::endl;
    fout << "# type: sparse matrix" << std::endl;
    fout << "# nnz: " << entries.size() << std::endl;
    fout << "# rows: " << rows << std::endl;
    fout << "# columns: " << cols << std::endl;
    //fout << fixed;
    fout << setprecision(9) << endl;
    for (vector<TripletEntry>::const_iterator it = entries.begin(); it != entries.end(); ++it) {
      const TripletEntry& entry = *it;
      fout << entry.r+1 << " " << entry.c+1 << " " << entry.x << std::endl;
    }
    return fout.good();
  }

} // end namespace
