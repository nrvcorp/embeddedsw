#ifndef PCIE_HPP
#define PCIE_HPP

#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#define RW_MAX_SIZE 0x7ffff000

class PCIe {
private:
    // PCIe connection
    const char *c2h_dev;
    const char *h2c_dev;
    int c2h_fd;
    int h2c_fd;

public:
    PCIe(const char* c2h_dev, const char* h2c_dev)
        : c2h_dev(c2h_dev), h2c_dev(h2c_dev), c2h_fd(-1), h2c_fd(-1)
    {
        // Connect PCIe
        c2h_fd = open(c2h_dev, O_RDWR);
        if (c2h_fd < 0) {
            fprintf(stderr, "unable to open device %s, %d.\r\n", c2h_dev, c2h_fd);
            perror("open device");
        } else {
            printf("%s connection success\r\n", c2h_dev);
        }

        h2c_fd = open(h2c_dev, O_RDWR);
        if (h2c_fd < 0) {
            fprintf(stderr, "unable to open device %s, %d.\r\n", h2c_dev, h2c_fd);
            perror("open device");
        } else {
            printf("%s connection success\r\n", h2c_dev);
        }
    }

    // c2h transfer
    ssize_t c2h(char *buffer, uint64_t size, uint64_t base)
    {
        ssize_t rc;
        uint64_t count = 0;
        char *buf = buffer;
        off_t offset = base;
        int loop = 0;

        while (count < size) {
            uint64_t bytes = size - count;
            if (bytes > RW_MAX_SIZE)
                bytes = RW_MAX_SIZE;

            if (offset) {
                rc = lseek(c2h_fd, offset, SEEK_SET);
                if (rc != offset) {
                    fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n",
                        c2h_dev, rc, offset);
                    perror("seek file");
                    return -EIO;
                }
            }

            // Read data from file into memory buffer
            rc = read(c2h_fd, buf, bytes);
            if (rc < 0) {
                fprintf(stderr, "%s, read 0x%lx @ 0x%lx failed %ld.\n",
                    c2h_dev, bytes, offset, rc);
                perror("read file");
                return -EIO;
            }

            count += rc;
            if (rc != bytes) {
                fprintf(stderr, "%s, read underflow 0x%lx/0x%lx @ 0x%lx.\n",
                    c2h_dev, rc, bytes, offset);
                break;
            }

            buf += bytes;
            offset += bytes;
            loop++;
        }

        if (count != size && loop)
            fprintf(stderr, "%s, read underflow 0x%lx/0x%lx.\n",
                c2h_dev, count, size);
        return count;
    }

    // h2c transfer
    ssize_t h2c(char *buffer, uint64_t size, uint64_t base)
    {
        ssize_t rc;
        uint64_t count = 0;
        char *buf = buffer;
        off_t offset = base;
        int loop = 0;

        while (count < size) {
            uint64_t bytes = size - count;
            if (bytes > RW_MAX_SIZE)
                bytes = RW_MAX_SIZE;

            if (offset) {
                rc = lseek(h2c_fd, offset, SEEK_SET);
                if (rc != offset) {
                    fprintf(stderr, "%s, seek off 0x%lx != 0x%lx.\n",
                        h2c_dev, rc, offset);
                    perror("seek file");
                    return -EIO;
                }
            }

            // Write data to file from memory buffer
            rc = write(h2c_fd, buf, bytes);
            if (rc < 0) {
                fprintf(stderr, "%s, write 0x%lx @ 0x%lx failed %ld.\n",
                    h2c_dev, bytes, offset, rc);
                perror("write file");
                return -EIO;
            }

            count += rc;
            if (rc != bytes) {
                fprintf(stderr, "%s, write underflow 0x%lx/0x%lx @ 0x%lx.\n",
                    h2c_dev, rc, bytes, offset);
                break;
            }
            buf += bytes;
            offset += bytes;
            loop++;
        }

        if (count != size && loop)
            fprintf(stderr, "%s, write underflow 0x%lx/0x%lx.\n",
                h2c_dev, count, size);

        return count;
    }

    // Destructor to clean up file descriptors
    ~PCIe() {
        if (c2h_fd >= 0) {
            close(c2h_fd);
        }
        if (h2c_fd >= 0) {
            close(h2c_fd);
        }
    }
};

#endif // PCIE_HPP
