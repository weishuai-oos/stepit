#include <memory>

#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <stepit/pyutils/npz_reader.h>

namespace stepit {
namespace field {
NpzReader::NpzReader(const std::string &path) {
  fs::path resolved_path(path);
  if (resolved_path.is_relative()) resolved_path = fs::absolute(resolved_path);
  if (not fs::exists(resolved_path)) {
    resolved_path.replace_extension(".npz");
    STEPIT_ASSERT(fs::exists(resolved_path), "File '{}' does not exist.", path);
  }

  std::unique_ptr<pybind11::scoped_interpreter> guard;
  if (not Py_IsInitialized()) {
    guard = std::make_unique<pybind11::scoped_interpreter>();
  }
  auto numpy = pybind11::module_::import("numpy");
  auto data  = numpy.attr("load")(fs::canonical(resolved_path).string());
  auto files = data.attr("files").cast<std::vector<std::string>>();
  for (const auto &file : files) {
    auto py_arr = data[file.c_str()].cast<pybind11::array>();
    auto py_buf = py_arr.request();

    NdArray array;
    array.shape.assign(py_buf.shape.begin(), py_buf.shape.end());
    array.dtype    = pybind11::str(py_arr.dtype()).cast<std::string>();
    array.nbytes   = py_buf.size * py_buf.itemsize;
    array.itemsize = py_buf.itemsize;
    array.ptr      = new char[array.nbytes];
    std::memcpy(array.ptr, py_buf.ptr, array.nbytes);

    insert(file, std::move(array));
  }
}

STEPIT_REGISTER_DATALOADER(npz, kDefPriority, DataLoader::make<NpzReader>);
}  // namespace field
}  // namespace stepit
