
#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <arrow/api.h>
#include <arrow/io/file.h>
#include <arrow/ipc/api.h>
#include <arrow/util/compression.h>
#include <unordered_map>
#include <vector>

class DataLogger {
public:
    DataLogger() = default;
    virtual ~DataLogger();

    // Add fields dynamically before opening the file
    void add_field(const std::string& name, std::shared_ptr<arrow::DataType> type);

    // Open the file after schema is defined
    void open_file(const std::string& filename);

    // Set value of a specific column in the current row (Overloaded for different types)
    void set_value(const std::string& column_name, int64_t value);
    void set_value(const std::string& column_name, int32_t value);
    void set_value(const std::string& column_name, int16_t value);
    void set_value(const std::string& column_name, int8_t value);
    void set_value(const std::string& column_name, double value);
    void set_value(const std::string& column_name, const std::string& value);


    void save_row();


private:
    std::vector<std::shared_ptr<arrow::Field>> fields_;
    std::shared_ptr<arrow::Schema> schema_;
    std::shared_ptr<arrow::io::OutputStream> outfile_;
    std::shared_ptr<arrow::ipc::RecordBatchWriter> writer_;
    std::unordered_map<std::string, size_t> column_indices_;  // Column name → index
    std::unordered_map<std::string, std::variant<int64_t, int32_t, int16_t, int8_t, double, std::string>> row_values_;  // Row values
    bool file_opened_ = false;

    void check_column(const std::string& column_name);
    void reset_row();
};


#endif // DATA_LOGGER_H

