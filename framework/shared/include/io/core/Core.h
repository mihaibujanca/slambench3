/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_CORE_H
#define IO_CORE_H

#include <aio.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <mutex>
#include <condition_variable>

namespace slambench {
	namespace io {
		namespace core {
			
			size_t FileSize(FILE *file);
			
			class MappedFile {
			public:
				MappedFile(void *mapped_ptr, size_t size) : ptr_(mapped_ptr), size_(size) {}
				MappedFile(MappedFile && other) noexcept : ptr_(other.ptr_), size_(other.size_) {}
				~MappedFile() { if (ptr_) munmap(ptr_, size_); }
				
				const void *Get() { return ptr_; }
				size_t Size() { return size_; }
			private:
				void *ptr_;
				size_t size_;
			};
			
			MappedFile ReadFile(const std::string &filename);
		}
	}
}

#endif /* IO_CORE_H */
