#include "airi/uccn.h"

#include "mpack/mpack.h"

static void *drive_state_allocate(const struct uccn_record_typesupport_s * ts) {
  static struct drive_state_s buffer;
  (void)ts;
  return &buffer;
}

static ssize_t drive_state_serialize(const struct uccn_record_typesupport_s * ts,
                                     struct drive_state_s * content,
                                     struct buffer_head_s * blob) {
  mpack_writer_t writer;
  (void)ts;
  mpack_writer_init(&writer, blob->data, blob->size);
  mpack_write_i32(&writer, content->left_encoder.ticks);
  mpack_write_i32(&writer, content->right_encoder.ticks);
  blob->length = mpack_writer_buffer_used(&writer);
  if (mpack_writer_destroy(&writer) != mpack_ok) {
    return -1;
  }
  return blob->length;
}

static ssize_t drive_state_deserialize(const struct uccn_record_typesupport_s * ts,
                                       const struct buffer_head_s * blob,
                                       struct drive_state_s * content) {
  mpack_reader_t reader;
  (void)ts;
  mpack_reader_init_data(&reader, blob->data, blob->length);
  content->left_encoder.ticks = mpack_expect_i32(&reader);
  content->right_encoder.ticks = mpack_expect_i32(&reader);
  if (mpack_reader_destroy(&reader) != mpack_ok) {
    return -1;
  }
  return blob->length;
}

static const struct uccn_record_typesupport_s g_drive_state_typesupport = {
  .allocate = (uccn_record_allocate_fn)drive_state_allocate,
  .serialize = (uccn_record_serialize_fn)drive_state_serialize,
  .deserialize = (uccn_record_deserialize_fn)drive_state_deserialize,
};

const struct uccn_record_typesupport_s * get_drive_state_typesupport(void) {
  return &g_drive_state_typesupport;
}
