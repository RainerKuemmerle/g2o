// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "edge_se2_segment2d_pointLine.h"

#ifdef WINDOWS
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

namespace g2o {

  EdgeSE2Segment2DPointLine::EdgeSE2Segment2DPointLine() :
    BaseBinaryEdge<3, Vector3D, VertexSE2, VertexSegment2D>()
  {
    _pointNum = 0;
  }

  bool EdgeSE2Segment2DPointLine::read(std::istream& is)
  {
    is >> _pointNum;
    for (size_t i = 0; i < 3 ; i++)
      is >> _measurement[i];
    for (size_t i = 0; i < 3 ; i++)
      for (size_t j = i; j < 3 ; j++) {
        is >> _information (i,j);
        _information (j,i) = _information (i,j);
      }
    return true;
  }

  bool EdgeSE2Segment2DPointLine::write(std::ostream& os) const
  {
    os << _pointNum << " ";
    for (size_t i = 0; i < 3 ; i++)
      os << _measurement[i] << " ";
    for (size_t i = 0; i < 3 ; i++)
      for (size_t j = i; j < 3 ; j++) {
        os << _information (i,j) << " ";
      }
    return os.good();
  }

} // end namespace
