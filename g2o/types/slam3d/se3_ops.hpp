  Matrix3d skew(const Vector3d&v)
  {
    Matrix3d m;
    m.fill(0.);
    m(0,1)  = -v(2);
    m(0,2)  =  v(1);
    m(1,2)  = -v(0);
    m(1,0)  =  v(2);
    m(2,0) = -v(1);
    m(2,1) = v(0);
    return m;
  }

  Vector3d deltaR(const Matrix3d& R)
  {
    Vector3d v;
    v(0)=R(2,1)-R(1,2);
    v(1)=R(0,2)-R(2,0);
    v(2)=R(1,0)-R(0,1);
    return v;
  }

  Vector2d project(const Vector3d& v)
  {
    Vector2d res;
    res(0) = v(0)/v(2);
    res(1) = v(1)/v(2);
    return res;
  }

  Vector3d project(const Vector4d& v)
  {
    Vector3d res;
    res(0) = v(0)/v(3);
    res(1) = v(1)/v(3);
    res(2) = v(2)/v(3);
    return res;
  }

  Vector3d unproject(const Vector2d& v)
  {
    Vector3d res;
    res(0) = v(0);
    res(1) = v(1);
    res(2) = 1;
    return res;
  }

  Vector4d unproject(const Vector3d& v)
  {
    Vector4d res;
    res(0) = v(0);
    res(1) = v(1);
    res(2) = v(2);
    res(3) = 1;
    return res;
  }


