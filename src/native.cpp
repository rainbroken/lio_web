// PointCloud2 and immutable archive kernels. No ROS or Python calls while
// processing points: the service retains ownership and schedules these jobs.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
#include <unistd.h>

namespace py = pybind11;
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
struct Point { double x, y, z, intensity; uint32_t rgb; };
struct Key {
  int64_t x, y, z;
  bool operator==(const Key& b) const { return x==b.x && y==b.y && z==b.z; }
};
struct Hash {
  size_t operator()(const Key& k) const {
    size_t h = std::hash<int64_t>{}(k.x);
    for (auto v : {k.y, k.z}) h ^= std::hash<int64_t>{}(v) + 0x9e3779b9 + (h<<6) + (h>>2);
    return h;
  }
};
static bool finite(const Point& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
static int64_t voxel(double v, double size) {
  double q = std::floor(v/size);
  if (!std::isfinite(q) || q < -0x1p63 || q >= 0x1p63)
    throw std::invalid_argument("coordinate outside voxel index range");
  return static_cast<int64_t>(q);
}
static uint8_t strength(double v, bool rounded) {
  if (!std::isfinite(v)) return 0;
  if (v >= 0 && v <= 1) v *= 255;
  return static_cast<uint8_t>(std::clamp(rounded ? std::nearbyint(v) : v, 0., 255.));
}
static uint64_t bits(const uint8_t* p, int size, bool big) {
  uint64_t value=0;
  for (int i=0; i<size; ++i) value |= uint64_t(p[i]) << (8*(big ? size-1-i : i));
  return value;
}
static int type_size(int type) {
  switch(type) { case 1: case 2: return 1; case 3: case 4: return 2;
    case 5: case 6: case 7: return 4; case 8: return 8; default: return 0; }
}
static double number(const uint8_t* p, int type, bool big) {
  auto u = bits(p, type_size(type), big);
  switch(type) {
    case 1: return static_cast<int8_t>(u); case 2: return u;
    case 3: return static_cast<int16_t>(u); case 4: return u;
    case 5: return static_cast<int32_t>(u); case 6: return u;
    case 7: { uint32_t v=u; float f; std::memcpy(&f,&v,4); return f; }
    case 8: { double f; std::memcpy(&f,&u,8); return f; }
    default: return 0;
  }
}
static void integer(std::string& out, uint64_t v, int bytes) {
  for(int i=0;i<bytes;++i) out.push_back(static_cast<char>((v>>(8*i))&255));
}
static void floating(std::string& out, double v) {
  float f=static_cast<float>(v); uint32_t u; std::memcpy(&u,&f,4); integer(out,u,4);
}
static std::string encode(const std::vector<Point>& points, const std::string& magic,
                          std::array<double,3> origin, double scale, bool rounded) {
  if (magic.size()!=4) throw std::invalid_argument("magic must be four bytes");
  std::string out; out.reserve(24+10*points.size()); out += magic;
  integer(out,points.size(),4); floating(out,scale);
  for(auto v:origin) floating(out,v);
  for(const auto& p:points) {
    std::array<double,3> xyz{p.x,p.y,p.z};
    for(int k=0;k<3;++k) {
      auto q=static_cast<int16_t>(std::clamp(std::nearbyint((xyz[k]-origin[k])/scale),-32768.,32767.));
      integer(out,static_cast<uint16_t>(q),2);
    }
    integer(out,(p.rgb>>16)&255,1); integer(out,(p.rgb>>8)&255,1);
    integer(out,p.rgb&255,1); integer(out,strength(p.intensity,rounded),1);
  }
  return out;
}
static std::array<double,3> minimum(const std::vector<Point>& points) {
  std::array<double,3> o{points[0].x,points[0].y,points[0].z};
  for(auto& p:points) { o[0]=std::min(o[0],p.x); o[1]=std::min(o[1],p.y); o[2]=std::min(o[2],p.z); }
  return o;
}
static double overview_scale(const std::vector<Point>& points, const std::array<double,3>& origin) {
  double extent=0;
  for(auto& p:points) extent=std::max({extent,p.x-origin[0],p.y-origin[1],p.z-origin[2]});
  return std::max(.01,extent/32760);
}

// Fields are offset/type pairs for x,y,z,intensity,rgb; -1 means absent.
static py::tuple cloud(py::buffer raw, size_t width, size_t height, size_t point_step,
                       size_t row_step, bool big, const std::vector<std::array<int,2>>& fields,
                       size_t limit, double size, const std::string& magic) {
  if (!limit || !std::isfinite(size) || size<=0) throw std::invalid_argument("invalid sampling parameters");
  auto info=raw.request();
  if (info.ndim!=1 || info.itemsize!=1 || info.strides[0]!=1)
    throw std::invalid_argument("cloud data must be a contiguous byte buffer");
  if(fields.size()!=5) throw std::invalid_argument("invalid field descriptors");
  if (!point_step || !width || !height) return py::make_tuple(py::bytes(),0);
  for(size_t i=0;i<fields.size();++i) {
    auto f=fields[i];
    if(f[0]<0 && i>=3) continue;
    auto n=type_size(f[1]);
    if(f[0]<0 || !n || size_t(f[0])+n>point_step)
      throw std::invalid_argument("field outside point record");
  }
  if(width>std::numeric_limits<size_t>::max()/point_step || row_step<width*point_step ||
     height>std::numeric_limits<size_t>::max()/row_step || size_t(info.size)<height*row_step)
    throw std::invalid_argument("truncated cloud or invalid row layout");
  std::vector<Point> points; std::unordered_set<Key,Hash> seen; std::string out;
  {
    py::gil_scoped_release release;
    size_t total=width*height, budget=limit>std::numeric_limits<size_t>::max()/4 ? total : limit*4;
    size_t step=std::max(size_t(1),total/budget+(total%budget!=0));
    points.reserve(std::min(total, budget)); seen.reserve(std::min(total,budget));
    auto data=static_cast<const uint8_t*>(info.ptr);
    for(size_t i=0;i<total;i+=step) {
      auto p=data+(i/width)*row_step+(i%width)*point_step;
      auto read=[&](int k) { auto f=fields[k]; return f[0]<0 ? 0. : number(p+f[0],f[1],big); };
      Point v{read(0),read(1),read(2),0,0}; if(!finite(v)) continue;
      Key key{voxel(v.x,size),voxel(v.y,size),voxel(v.z,size)};
      if(!seen.insert(key).second) continue;
      v.intensity=read(3); auto rgb=fields[4];
      if(rgb[0]>=0 && (rgb[1]==6 || rgb[1]==7)) v.rgb=bits(p+rgb[0],4,big)&0xffffff;
      else { uint32_t s=strength(v.intensity,true); v.rgb=(s<<16)|(s<<8)|s; }
      points.push_back(v);
    }
    if(points.size()>limit) {
      std::vector<Point> sampled; sampled.reserve(limit);
      double stride=double(points.size())/limit;
      for(size_t i=0;i<limit;++i) sampled.push_back(points[std::min(points.size()-1,size_t(i*stride))]);
      points.swap(sampled);
    }
    if(!points.empty()) out=encode(points,magic,minimum(points),std::max(.01,size),true);
  }
  return py::make_tuple(py::bytes(out),points.size());
}

static void pread_exact(int fd, uint8_t* buffer, size_t count, uint64_t offset) {
  if(offset>uint64_t(std::numeric_limits<off_t>::max())-count)
    throw std::invalid_argument("archive offset out of range");
  size_t done=0;
  while(done<count) {
    auto n=::pread(fd,buffer+done,count-done,static_cast<off_t>(offset+done));
    if(n<0 && errno==EINTR) continue;
    if(n<0) throw std::runtime_error(std::string("archive read failed: ")+std::strerror(errno));
    if(!n) throw std::invalid_argument("archive prefix changed or truncated");
    done+=n;
  }
}
static void record(const uint8_t* raw, double* out) {
  for(int i=0;i<4;++i) out[i]=number(raw+4*i,7,false);
  out[4]=bits(raw+16,4,false);
}
// At most 64 KiB of temporary I/O storage, coalescing reasonably close samples.
static Array samples(int fd, uint64_t start, size_t count, size_t step) {
  if(!step) throw std::invalid_argument("sample step must be positive");
  constexpr auto maximum=uint64_t(std::numeric_limits<off_t>::max());
  if(start>maximum-20 || (count && uint64_t(count-1)>(maximum-start-20)/20))
    throw std::invalid_argument("archive range out of bounds");
  size_t n=count/step+(count%step!=0);
  Array result({static_cast<py::ssize_t>(n),py::ssize_t(5)}); auto out=result.mutable_data();
  {
    py::gil_scoped_release release;
    std::vector<uint8_t> buffer(65536);
    size_t batch=step<=32 ? std::max(size_t(1),size_t(65536/20)/step) : 1;
    for(size_t i=0;i<n;) {
      size_t take=std::min(batch,n-i), bytes=((take-1)*step+1)*20;
      pread_exact(fd,buffer.data(),bytes,start+uint64_t(i)*step*20);
      for(size_t j=0;j<take;++j) record(buffer.data()+j*step*20,out+(i+j)*5);
      i+=take;
    }
  }
  return result;
}
static Array offsets(int fd, py::array_t<int64_t,py::array::c_style|py::array::forcecast> indices) {
  auto data=indices.request();
  if(data.ndim!=1) throw std::invalid_argument("offsets must be a vector");
  auto values=static_cast<int64_t*>(data.ptr); size_t count=data.size;
  Array result({static_cast<py::ssize_t>(count),py::ssize_t(5)}); auto out=result.mutable_data();
  {
    py::gil_scoped_release release; std::vector<uint8_t> buffer(65536);
    for(size_t i=0;i<count;) {
      if(values[i]<0) throw std::invalid_argument("negative archive offset");
      size_t end=i+1;
      while(end<count && values[end]>=values[end-1] && values[end]-values[end-1]>=20 &&
            values[end]-values[i]<=65516 &&
            values[end]-values[end-1]<=640) ++end;
      pread_exact(fd,buffer.data(),size_t(values[end-1]-values[i])+20,values[i]);
      for(size_t j=i;j<end;++j) record(buffer.data()+values[j]-values[i],out+j*5);
      i=end;
    }
  }
  return result;
}
static std::vector<Point> from_array(const double* data, size_t count) {
  std::vector<Point> points; points.reserve(count);
  for(size_t i=0;i<count;++i) {
    auto p=data+i*5;
    if(!std::isfinite(p[4]) || p[4]<0 || p[4]>double(UINT32_MAX))
      throw std::invalid_argument("invalid RGB integer");
    Point v{p[0],p[1],p[2],p[3],static_cast<uint32_t>(p[4])};
    if(finite(v)) points.push_back(v);
  }
  return points;
}
static py::bytes encode_array(Array data, const std::string& magic, py::object origin) {
  auto a=data.request(); if(a.ndim!=2 || a.shape[1]!=5) throw std::invalid_argument("expected Nx5 points");
  bool fixed=!origin.is_none(); std::array<double,3> o{};
  if(fixed) o=origin.cast<std::array<double,3>>();
  for(double v:o) if(!std::isfinite(v)) throw std::invalid_argument("invalid point origin");
  std::string out;
  {
    py::gil_scoped_release release;
    auto points=from_array(static_cast<double*>(a.ptr),a.shape[0]);
    if(!fixed && points.empty()) throw std::invalid_argument("empty optimized overview");
    if(!fixed) o=minimum(points);
    out=encode(points,magic,o,fixed ? .01 : overview_scale(points,o),false);
  }
  return py::bytes(out);
}
static Array transform_points(Array data, Array matrix) {
  auto a=data.request(), t=matrix.request();
  if(a.ndim!=2 || a.shape[1]!=3 || t.ndim!=2 || t.shape[0]!=3 || t.shape[1]!=4)
    throw std::invalid_argument("expected Nx3 coordinates and 3x4 transform");
  Array result({a.shape[0],py::ssize_t(3)});
  auto raw=static_cast<double*>(a.ptr), m=static_cast<double*>(t.ptr), out=result.mutable_data();
  {
    py::gil_scoped_release release;
    for(py::ssize_t i=0;i<a.shape[0];++i)
      for(int k=0;k<3;++k)
        out[3*i+k]=raw[3*i]*m[4*k]+raw[3*i+1]*m[4*k+1]+raw[3*i+2]*m[4*k+2]+m[4*k+3];
  }
  return result;
}
class TileBuilder {
  std::array<double,3> origin_;
  size_t limit_;
  std::unordered_set<Key,Hash> seen_;
  std::vector<Point> points_;
  std::mutex mutex_;
public:
  TileBuilder(std::array<double,3> origin, size_t limit):origin_(origin),limit_(limit) {
    if(!limit || limit>100000) throw std::invalid_argument("invalid tile point limit");
    for(double v:origin) if(!std::isfinite(v)) throw std::invalid_argument("invalid tile origin");
  }
  bool append(Array data, Array transform) {
    auto a=data.request(), t=transform.request();
    if(a.ndim!=2 || a.shape[1]!=5 || t.ndim!=2 || t.shape[0]!=3 || t.shape[1]!=4)
      throw std::invalid_argument("invalid tile arrays");
    auto raw=static_cast<double*>(a.ptr), m=static_cast<double*>(t.ptr);
    py::gil_scoped_release release;
    std::lock_guard<std::mutex> guard(mutex_);
    for(size_t i=0;i<size_t(a.shape[0]) && points_.size()<limit_;++i) {
      auto p=raw+i*5; double xyz[3];
      for(int k=0;k<3;++k) xyz[k]=p[0]*m[4*k]+p[1]*m[4*k+1]+p[2]*m[4*k+2]+m[4*k+3];
      Point v{xyz[0],xyz[1],xyz[2],p[3],0};
      if(!finite(v)) continue;
      if(v.x<origin_[0] || v.x>=origin_[0]+5 || v.y<origin_[1] || v.y>=origin_[1]+5 ||
         v.z<origin_[2] || v.z>=origin_[2]+5) continue;
      if(!std::isfinite(p[4]) || p[4]<0 || p[4]>double(UINT32_MAX))
        throw std::invalid_argument("invalid RGB integer");
      v.rgb=static_cast<uint32_t>(p[4]);
      if(seen_.insert({voxel(v.x,.01),voxel(v.y,.01),voxel(v.z,.01)}).second) points_.push_back(v);
    }
    return points_.size()>=limit_;
  }
  py::bytes payload() {
    std::string out;
    {
      py::gil_scoped_release release;
      std::lock_guard<std::mutex> guard(mutex_);
      out=encode(points_,"LIT3",origin_,.01,false);
    }
    return py::bytes(out);
  }
};
PYBIND11_MODULE(_native,m) {
  m.doc()="C++17 point-cloud encoding and bounded archive I/O kernels";
  m.def("encode_cloud",&cloud);
  m.def("read_samples",&samples);
  m.def("read_offsets",&offsets);
  m.def("encode_points",&encode_array,py::arg("points"),py::arg("magic"),py::arg("origin")=py::none());
  m.def("transform",&transform_points);
  py::class_<TileBuilder>(m,"TileBuilder")
    .def(py::init<std::array<double,3>,size_t>()).def("append",&TileBuilder::append).def("payload",&TileBuilder::payload);
}
