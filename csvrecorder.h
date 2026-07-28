#ifndef CSVRECORDER_H
#define CSVRECORDER_H

#include <QFile>
#include <QString>
#include <QTextStream>

// Records joint samples to a long-format CSV: one row per sample. Long format is
// used because samples arrive per-joint (round-robin), not synchronized across
// joints, so a wide one-column-per-joint layout would not line up.
//
//   time_s,id,name,signal,value
//   0.812,1,HLS3955,position,3083
//
// Usage: start(path, signal) -> write(...) per sample -> stop().
class CsvRecorder
{
public:
    ~CsvRecorder() { stop(); }

    bool isRecording() const { return file_.isOpen(); }

    bool start(const QString &path, const QString &signal)
    {
        stop();
        file_.setFileName(path);
        if(!file_.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
            return false;
        stream_.setDevice(&file_);
        signal_ = signal;
        stream_ << "time_s,id,name,signal,value\n";
        return true;
    }

    void write(double t, int id, const QString &name, int value)
    {
        if(!file_.isOpen())
            return;
        stream_ << QString::number(t, 'f', 3) << ','
                << id << ','
                << name << ','
                << signal_ << ','
                << value << '\n';
    }

    // Set/refresh the signal label written into subsequent rows.
    void setSignal(const QString &signal) { signal_ = signal; }

    void stop()
    {
        if(file_.isOpen())
        {
            stream_.flush();
            file_.close();
        }
    }

private:
    QFile       file_;
    QTextStream stream_;
    QString     signal_;
};

#endif // CSVRECORDER_H
