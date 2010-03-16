/*
 * Copyright (C) 2004 Boston University
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 2 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <qapplication.h>
#include <qcombobox.h>
#include <qfiledialog.h>
#include <qgroupbox.h>
#include <qhbox.h>
#include <qhgroupbox.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qlineedit.h>
#include <qlistbox.h>
#include <qmessagebox.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qstringlist.h>
#include <qvbox.h>
#include <qvgroupbox.h>
#include <qwaitcondition.h>

#include <compiler.h>
#include <debug.h>
#include <main_window.h>
#include <sstream>
#include <workspace.h>
#include <data_recorder.h>

#define QFileExistsEvent            (QEvent::User+0)
#define QNoFileOpenEvent            (QEvent::User+1)
#define QSetFileNameEditEvent       (QEvent::User+2)
#define QDisableGroupsEvent         (QEvent::User+3)
#define QEnableGroupsEvent          (QEvent::User+4)

struct param_hdf_t {
    long long index;
    double value;
};

namespace {

    void buildBlockPtrList(IO::Block *block,void *arg) {
        std::vector<IO::Block *> *list = reinterpret_cast<std::vector<IO::Block *> *>(arg);
        list->push_back(block);
    };

    struct FileExistsEventData {
        QString filename;
        int response;
        QWaitCondition done;
    };

    struct SetFileNameEditEventData {
        QString filename;
        QWaitCondition done;
    };

    class InsertChannelEvent : public RT::Event {

    public:

        InsertChannelEvent(bool &,RT::List<DataRecorder::Channel> &,RT::List<DataRecorder::Channel>::iterator,DataRecorder::Channel &);
        ~InsertChannelEvent(void);

        int callback(void);

    private:

        bool &recording;
        RT::List<DataRecorder::Channel> &channels;
        RT::List<DataRecorder::Channel>::iterator end;
        DataRecorder::Channel &channel;

    }; // class InsertChannelEvent

    class RemoveChannelEvent : public RT::Event {

    public:

        RemoveChannelEvent(bool &,RT::List<DataRecorder::Channel> &,DataRecorder::Channel &);
        ~RemoveChannelEvent(void);

        int callback(void);

    private:

        bool &recording;
        RT::List<DataRecorder::Channel> &channels;
        DataRecorder::Channel &channel;

    }; // class RemoveChannelEvent

    class OpenFileEvent : public RT::Event {

    public:

        OpenFileEvent(QString &,NoCopyFifo &);
        ~OpenFileEvent(void);

        int callback(void);

    private:

        QString &filename;
        NoCopyFifo &fifo;

    }; // class OpenFileEvent

    class StartRecordingEvent : public RT::Event {

    public:

        StartRecordingEvent(bool &,NoCopyFifo &);
        ~StartRecordingEvent(void);

        int callback(void);

    private:

        bool &recording;
        NoCopyFifo &fifo;

    }; // class StartRecordingEvent

    class StopRecordingEvent : public RT::Event {

    public:

        StopRecordingEvent(bool &,NoCopyFifo &);
        ~StopRecordingEvent(void);

        int callback(void);

    private:

        bool &recording;
        NoCopyFifo &fifo;

    }; //class StopRecordingEvent

    class AsyncDataEvent : public RT::Event {

    public:

        AsyncDataEvent(const double *,size_t,NoCopyFifo &);
        ~AsyncDataEvent(void);

        int callback(void);

    private:

        const double *data;
        size_t size;
        NoCopyFifo &fifo;

    }; // class AsyncDataEvent

    class DoneEvent : public RT::Event {

    public:

        DoneEvent(NoCopyFifo &);
        ~DoneEvent(void);

        int callback(void);

    private:

        NoCopyFifo &fifo;

    }; // class DoneEvent

}; // namespace

InsertChannelEvent::InsertChannelEvent(bool &r,RT::List<DataRecorder::Channel> & l,RT::List<DataRecorder::Channel>::iterator e,DataRecorder::Channel &c)
    : recording(r), channels(l), end(e), channel(c) {}

InsertChannelEvent::~InsertChannelEvent(void) {}

int InsertChannelEvent::callback(void) {
    if(recording)
        return -1;

    channels.insertRT(end,channel);
    return 0;
}

RemoveChannelEvent::RemoveChannelEvent(bool &r,RT::List<DataRecorder::Channel> & l,DataRecorder::Channel &c)
    : recording(r), channels(l), channel(c) {}

RemoveChannelEvent::~RemoveChannelEvent(void) {}

int RemoveChannelEvent::callback(void) {
    if(recording)
        return -1;

    channels.removeRT(channel);
    return 0;
}

OpenFileEvent::OpenFileEvent(QString &n,NoCopyFifo &f)
    : filename(n), fifo(f) {}

OpenFileEvent::~OpenFileEvent(void) {}

int OpenFileEvent::callback(void) {
    DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)+filename.length()+1));

    if(!token)
        return -1;

    token->type = DataRecorder::OPEN;
    token->size = filename.length()+1;
    token->time = RT::OS::getTime();

    char *name = reinterpret_cast<char *>(token+1);
    strcpy(name,filename.latin1());

    fifo.writeDone();

    return 0;
}

StartRecordingEvent::StartRecordingEvent(bool &r,NoCopyFifo &f)
    : recording(r), fifo(f) {}

StartRecordingEvent::~StartRecordingEvent(void) {}

int StartRecordingEvent::callback(void) {
    DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)));

    if(!token)
        return -1;

    recording = true;

    token->type = DataRecorder::START;
    token->size = 0;
    token->time = RT::OS::getTime();

    fifo.writeDone();

    return 0;
}

StopRecordingEvent::StopRecordingEvent(bool &r,NoCopyFifo &f)
    : recording(r), fifo(f) {}

StopRecordingEvent::~StopRecordingEvent(void) {}

int StopRecordingEvent::callback(void) {
    DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)));

    if(!token)
        return -1;

    recording = false;

    token->type = DataRecorder::STOP;
    token->size = 0;
    token->time = RT::OS::getTime();

    fifo.writeDone();

    return 0;
}

AsyncDataEvent::AsyncDataEvent(const double *d,size_t s,NoCopyFifo &f)
    : data(d), size(s), fifo(f) {}

AsyncDataEvent::~AsyncDataEvent(void) {}

int AsyncDataEvent::callback(void) {
    DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)+size*sizeof(double)));

    token->type = DataRecorder::ASYNC;
    token->size = size*sizeof(double);
    token->time = RT::OS::getTime();

    double *data_dest = reinterpret_cast<double *>(token+1);
    memcpy(data_dest,data,token->size);

    fifo.writeDone();

    return 1;
}

DoneEvent::DoneEvent(NoCopyFifo &f)
    : fifo(f) {}

DoneEvent::~DoneEvent(void) {}

int DoneEvent::callback(void) {
    DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)));

    if(!token)
        return -1;

    token->type = DataRecorder::DONE;
    token->size = 0;
    token->time = RT::OS::getTime();

    fifo.writeDone();

    return 0;
}

void DataRecorder::startRecording(void) {
    Event::Object event(Event::START_RECORDING_EVENT);

    if(RT::OS::isRealtime())
        Event::Manager::getInstance()->postEventRT(&event);
    else
        Event::Manager::getInstance()->postEvent(&event);
}

void DataRecorder::stopRecording(void) {
    Event::Object event(Event::STOP_RECORDING_EVENT);

    if(RT::OS::isRealtime())
        Event::Manager::getInstance()->postEventRT(&event);
    else
        Event::Manager::getInstance()->postEvent(&event);
}

void DataRecorder::postAsyncData(const double *data,size_t size) {
    Event::Object event(Event::ASYNC_DATA_EVENT);
    event.setParam("data",const_cast<double *>(data));
    event.setParam("size",&size);

    if(RT::OS::isRealtime())
        Event::Manager::getInstance()->postEventRT(&event);
    else
        Event::Manager::getInstance()->postEvent(&event);
}

DataRecorder::Channel::Channel(void) {}

DataRecorder::Channel::~Channel(void) {}

DataRecorder::Panel::Panel(QWidget *parent)
    : QWidget(parent,0,Qt::WStyle_NormalBorder|Qt::WDestructiveClose),
      RT::Thread(RT::Thread::MinimumPriority), fifo(1048576), recording(false) {
    setCaption(QString::number(getID())+" Data Recorder");

    QHBox *hbox;
    QVBox *vbox;

    QBoxLayout *layout = new QVBoxLayout(this);
    layout->setMargin(5);

    channelBox = new QHGroupBox("Channel Selection",this);
    channelBox->setInsideMargin(5);
    layout->addWidget(channelBox);

    vbox = new QVBox(channelBox);
    vbox->setMaximumHeight(125);

    hbox = new QHBox(vbox);
    (new QLabel("Block:",hbox))->setFixedWidth(75);
    blockList = new QComboBox(hbox);
    blockList->setFixedWidth(150);
    QObject::connect(blockList,SIGNAL(activated(int)),this,SLOT(buildChannelList(void)));

    hbox = new QHBox(vbox);
    (new QLabel("Type:",hbox))->setFixedWidth(75);
    typeList = new QComboBox(hbox);
    typeList->setFixedWidth(150);
    typeList->insertItem("Input");
    typeList->insertItem("Output");
    typeList->insertItem("Parameter");
    typeList->insertItem("State");
    typeList->insertItem("Event");
    QObject::connect(typeList,SIGNAL(activated(int)),this,SLOT(buildChannelList(void)));

    hbox = new QHBox(vbox);
    (new QLabel("Channel:",hbox))->setFixedWidth(75);
    channelList = new QComboBox(hbox);
    channelList->setFixedWidth(150);

    vbox = new QVBox(channelBox);
    vbox->setMaximumHeight(100);

    QPushButton *rButton = new QPushButton(">",vbox);
    rButton->setFixedWidth(rButton->height());
    QObject::connect(rButton,SIGNAL(pressed(void)),this,SLOT(insertChannel(void)));
    QPushButton *lButton = new QPushButton("<",vbox);
    lButton->setFixedWidth(lButton->height());
    QObject::connect(lButton,SIGNAL(pressed(void)),this,SLOT(removeChannel(void)));

    selectionBox = new QListBox(channelBox);

    sampleBox = new QVGroupBox("Sample Control",this);
    sampleBox->setInsideMargin(5);
    layout->addWidget(sampleBox);

    hbox = new QHBox(sampleBox);
    (new QLabel("Downsampling Rate:",hbox))->setFixedWidth(150);
    downsampleSpin = new QSpinBox(1,1000,1,hbox);
    QObject::connect(downsampleSpin,SIGNAL(valueChanged(int)),this,SLOT(updateDownsampleRate(int)));

    fileBox = new QVGroupBox("File Control",this);
    fileBox->setInsideMargin(5);
    layout->addWidget(fileBox);

    hbox = new QHBox(fileBox);
    hbox->setSpacing(2);
    (new QLabel("File:",hbox))->setFixedWidth(50);
    fileNameEdit = new QLineEdit(hbox);
    fileNameEdit->setReadOnly(true);
    QPushButton *fileChangeButton = new QPushButton("Change",hbox);
    QObject::connect(fileChangeButton,SIGNAL(clicked(void)),this,SLOT(changeDataFile(void)));

    hbox = new QHBox(this);
    layout->addWidget(hbox);

    startRecordButton = new QPushButton("Start Recording",hbox);
    QObject::connect(startRecordButton,SIGNAL(clicked(void)),this,SLOT(startRecordClicked(void)));
    stopRecordButton  = new QPushButton("Stop Recording",hbox);
    QObject::connect(stopRecordButton,SIGNAL(clicked(void)),this,SLOT(stopRecordClicked(void)));

    QPushButton *closeButton = new QPushButton("Close",hbox);
    QObject::connect(closeButton,SIGNAL(clicked(void)),this,SLOT(close(void)));

    resize(550,260);
    show();

    // Build initial block list
    IO::Connector::getInstance()->foreachBlock(buildBlockPtrList,&blockPtrList);
    for(std::vector<IO::Block *>::const_iterator i = blockPtrList.begin(),end = blockPtrList.end();i != end;++i)
        blockList->insertItem((*i)->getName()+" "+QString::number((*i)->getID()));

    // Build initial channel list
    buildChannelList();

    // Launch Recording Thread
    pthread_create(&thread,0,bounce,this);

    counter = 0;
    downsample_rate = 1;
    prev_input = 0.0;

    setActive(true);
}

DataRecorder::Panel::~Panel(void) {
    Plugin::getInstance()->removeDataRecorderPanel(this);

    setActive(false);

    DoneEvent event(fifo);
    while(RT::System::getInstance()->postEvent(&event));

    pthread_join(thread,0);

    for(RT::List<Channel>::iterator i = channels.begin(),end = channels.end();i != end;)
        delete &*(i++);
}

void DataRecorder::Panel::execute(void) {
    if(recording && !counter++) {
        void *buffer = fifo.write(sizeof(data_token_t)+channels.size()*sizeof(double));
        if(unlikely(!buffer))
            return;

        data_token_t *token = reinterpret_cast<data_token_t *>(buffer);
        double *data = reinterpret_cast<double *>(token+1);

        size_t n = 0;
        token->type = SYNC;
        token->size = channels.size()*sizeof(double);
        for(RT::List<Channel>::iterator i = channels.begin(),end = channels.end();i != end;++i)
            if(i->block) data[n++] = i->block->getValue(i->type,i->index);

        fifo.writeDone();
    }

    counter %= downsample_rate;
}

void DataRecorder::Panel::receiveEvent(const Event::Object *event) {
    if(event->getName() == Event::IO_BLOCK_INSERT_EVENT) {

        IO::Block *block = reinterpret_cast<IO::Block *>(event->getParam("block"));

        blockPtrList.push_back(block);
        blockList->insertItem(block->getName()+" "+QString::number(block->getID()));
        if(blockList->count() == 1)
            buildChannelList();

    } else if(event->getName() == Event::IO_BLOCK_REMOVE_EVENT) {

        IO::Block *block = reinterpret_cast<IO::Block *>(event->getParam("block"));
        QString name = block->getName()+" "+QString::number(block->getID());

        int n = 0;
        for(;n < blockList->count() && blockList->text(n) != name;++n);
        if(n < blockList->count())
            blockList->removeItem(n);
        blockPtrList.erase(blockPtrList.begin()+n);

        for(RT::List<Channel>::iterator i = channels.begin(),end = channels.end();i != end;++i)
            if(i->block == block)
                if(recording)
                    i->block = 0;

    } else if(event->getName() == Event::START_RECORDING_EVENT) {

        StartRecordingEvent e(recording,fifo);
        RT::System::getInstance()->postEvent(&e);

    } else if(event->getName() == Event::STOP_RECORDING_EVENT) {

        StopRecordingEvent e(recording,fifo);
        RT::System::getInstance()->postEvent(&e);

    } else if(event->getName() == Event::ASYNC_DATA_EVENT) {

        AsyncDataEvent e(reinterpret_cast<double *>(event->getParam("data")),
                             *reinterpret_cast<size_t *>(event->getParam("size")),fifo);
        RT::System::getInstance()->postEvent(&e);

    }
}

void DataRecorder::Panel::receiveEventRT(const Event::Object *event) {
    if(event->getName() == Event::START_RECORDING_EVENT) {
        DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)));

        if(!token)
            return;

        recording = true;

        token->type = DataRecorder::START;
        token->size = 0;
        token->time = RT::OS::getTime();

        fifo.writeDone();
    } else if(event->getName() == Event::STOP_RECORDING_EVENT) {
        DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)));

        if(!token)
            return;

        recording = false;

        token->type = DataRecorder::STOP;
        token->size = 0;
        token->time = RT::OS::getTime();

        fifo.writeDone();
    } else if(event->getName() == Event::ASYNC_DATA_EVENT) {
        size_t size = *reinterpret_cast<size_t *>(event->getParam("size"));

        DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)+size*sizeof(double)));

        token->type = DataRecorder::ASYNC;
        token->size = size*sizeof(double);
        token->time = RT::OS::getTime();

        double *data = reinterpret_cast<double *>(token+1);
        memcpy(data,event->getParam("data"),token->size);

        fifo.writeDone();
    } else if(event->getName() == Event::WORKSPACE_PARAMETER_CHANGE_EVENT) {
        DataRecorder::data_token_t *token = reinterpret_cast<DataRecorder::data_token_t *>(fifo.write(sizeof(DataRecorder::data_token_t)+sizeof(param_change_t)));

        token->type = DataRecorder::PARAM;
        token->size = sizeof(param_change_t);
        token->time = RT::OS::getTime();

        param_change_t *data = reinterpret_cast<param_change_t *>(token+1);
        data->id = reinterpret_cast<Settings::Object::ID>(event->getParam("object"));
        data->index = reinterpret_cast<size_t>(event->getParam("index"));
        data->step = file.idx;
        memcpy(&data->value,event->getParam("value"),sizeof(double));

        fifo.writeDone();
    }
}

void DataRecorder::Panel::buildChannelList(void) {
    channelList->clear();

    if(!blockList->count())
        return;

    IO::Block *block = blockPtrList[blockList->currentItem()];
    IO::flags_t type;
    switch(typeList->currentItem()) {
      case 0:
          type = Workspace::INPUT;
          break;
      case 1:
          type = Workspace::OUTPUT;
          break;
      case 2:
          type = Workspace::PARAMETER;
          break;
      case 3:
          type = Workspace::STATE;
          break;
      case 4:
          type = Workspace::EVENT;
          break;
      default:
          ERROR_MSG("DataRecorder::Panel::buildChannelList : invalid type selection\n");
          typeList->setCurrentItem(0);
          type = Workspace::INPUT;
    }

    for(size_t i = 0;i < block->getCount(type);++i)
        channelList->insertItem(block->getName(type,i));
}

void DataRecorder::Panel::changeDataFile(void) {
    QFileDialog fileDialog(this,NULL,true);
    fileDialog.setCaption("Select Data File");
    fileDialog.setMode(QFileDialog::AnyFile);

    QStringList filterList;
    filterList.push_back("HDF5 files (*.h5)");
    filterList.push_back("All files (*.*)");
    fileDialog.setFilters(filterList);

    fileDialog.exec();

    if(fileDialog.selectedFile() == "/" ||
       fileDialog.selectedFile().isNull() ||
       fileDialog.selectedFile().isEmpty())
        return;

    QString filename = fileDialog.selectedFile();

    if(!filename.lower().endsWith(QString(".h5")))
        filename += ".h5";

    OpenFileEvent event(filename,fifo);
    RT::System::getInstance()->postEvent(&event);
}

void DataRecorder::Panel::insertChannel(void) {
    if(!blockList->count() || !channelList->count())
        return;

    Channel *channel = new Channel();
    channel->block = blockPtrList[blockList->currentItem()];
    switch(typeList->currentItem()) {
      case 0:
          channel->type = Workspace::INPUT;
          break;
      case 1:
          channel->type = Workspace::OUTPUT;
          break;
      case 2:
          channel->type = Workspace::PARAMETER;
          break;
      case 3:
          channel->type = Workspace::STATE;
          break;
      case 4:
          channel->type = Workspace::EVENT;
          break;
      default:
          ERROR_MSG("DataRecorder::Panel::insertChannel : invalid type selection\n");
          typeList->setCurrentItem(0);
          channel->type = Workspace::INPUT;
    }
    channel->index = channelList->currentItem();

    channel->name.sprintf("%s %ld : %s",channel->block->getName().c_str(),channel->block->getID(),
                          channel->block->getName(channel->type,channel->index).c_str());

    InsertChannelEvent event(recording,channels,channels.end(),*channel);
    if(!RT::System::getInstance()->postEvent(&event))
        selectionBox->insertItem(channel->name);
}

void DataRecorder::Panel::removeChannel(void) {
    if(!selectionBox->count())
        return;

    for(RT::List<Channel>::iterator i = channels.begin(),end = channels.end();i != end;++i)
        if(i->name == selectionBox->currentText()) {
            RemoveChannelEvent event(recording,channels,*i);
            if(!RT::System::getInstance()->postEvent(&event))
                selectionBox->removeItem(selectionBox->currentItem());
            break;
        }
}

void DataRecorder::Panel::startRecordClicked(void) {
    StartRecordingEvent event(recording,fifo);
    RT::System::getInstance()->postEvent(&event);
}

void DataRecorder::Panel::stopRecordClicked(void) {
    StopRecordingEvent event(recording,fifo);
    RT::System::getInstance()->postEvent(&event);
}

void DataRecorder::Panel::updateDownsampleRate(int r) {
    downsample_rate = r;
}

void DataRecorder::Panel::customEvent(QCustomEvent *e) {
    if(e->type() == QFileExistsEvent) {
        FileExistsEventData *data = reinterpret_cast<FileExistsEventData *>(e->data());
        data->response = QMessageBox::information(this,"File exists","The file \""+data->filename+"\" already exists.",
                                                  "Append","Overwrite","Cancel",0,2);
        data->done.wakeAll();
    } else if(e->type() == QNoFileOpenEvent) {
        QMessageBox::critical(this,"Failed to start recording",
                              "No file has been opened for writing so recording could not be started.",
                              QMessageBox::Ok,QMessageBox::NoButton);
    } else if(e->type() == QSetFileNameEditEvent) {
        SetFileNameEditEventData *data = reinterpret_cast<SetFileNameEditEventData *>(e->data());
        fileNameEdit->setText(data->filename);
        data->done.wakeAll();
    } else if(e->type() == QDisableGroupsEvent) {
        channelBox->setEnabled(false);
        sampleBox->setEnabled(false);
    } else if(e->type() == QEnableGroupsEvent) {
        channelBox->setEnabled(true);
        sampleBox->setEnabled(true);
    }
}

void DataRecorder::Panel::doDeferred(const Settings::Object::State &s) {
    for(int i = 0;i < s.loadInteger("Num Channels");++i) {
        Channel *channel;
        IO::Block *block;
        std::ostringstream str;
        str << i;

        block = dynamic_cast<IO::Block *>(Settings::Manager::getInstance()->getObject(s.loadInteger(str.str()+" ID")));
        if(!block) continue;

        channel = new Channel();

        channel->block = block;
        channel->type = s.loadInteger(str.str()+" type");
        channel->index = s.loadInteger(str.str()+" index");
        channel->name.sprintf("%s %ld : %s",channel->block->getName().c_str(),channel->block->getID(),
                              channel->block->getName(channel->type,channel->index).c_str());

        channels.insert(channels.end(),*channel);
        selectionBox->insertItem(channel->name);
    }
}

void DataRecorder::Panel::doLoad(const Settings::Object::State &s) {
    if(s.loadInteger("Maximized"))
        showMaximized();
    else if(s.loadInteger("Minimized"))
        showMinimized();

    downsampleSpin->setValue(s.loadInteger("Downsample"));

    resize(s.loadInteger("W"),
           s.loadInteger("H"));
    parentWidget()->move(s.loadInteger("X"),
                         s.loadInteger("Y"));
}

void DataRecorder::Panel::doSave(Settings::Object::State &s) const {
    if(isMaximized())
        s.saveInteger("Maximized",1);
    else if(isMinimized())
        s.saveInteger("Minimized",1);

    QPoint pos = parentWidget()->pos();
    s.saveInteger("X",pos.x());
    s.saveInteger("Y",pos.y());
    s.saveInteger("W",width());
    s.saveInteger("H",height());

    s.saveInteger("Downsample",downsampleSpin->value());

    s.saveInteger("Num Channels",channels.size());
    size_t n = 0;
    for(RT::List<Channel>::const_iterator i = channels.begin(),end = channels.end();i != end;++i) {
        std::ostringstream str;
        str << n++;

        s.saveInteger(str.str()+" ID",i->block->getID());
        s.saveInteger(str.str()+" type",i->type);
        s.saveInteger(str.str()+" index",i->index);
    }
}

void *DataRecorder::Panel::bounce(void *param) {
    Panel *that = reinterpret_cast<Panel *>(param);
    if(that) that->processData();
    return 0;
}

void DataRecorder::Panel::processData(void) {
    enum {
        CLOSED,
        OPENED,
        RECORD,
    } state = CLOSED;

    data_token_t token;

    for(;;) {

        memcpy(&token,reinterpret_cast<data_token_t *>(fifo.read(sizeof(token))),sizeof(token));
        fifo.readDone();

        if(token.type == SYNC) {

            if(state == RECORD) {
                double *data = reinterpret_cast<double *>(fifo.read(token.size));
                H5PTappend(file.cdata,1,data);
                fifo.readDone();

                ++file.idx;
            }

        } else if(token.type == ASYNC) {

            if(state == RECORD) {

                double *data = reinterpret_cast<double *>(fifo.read(token.size));

                if(data) {
                    hsize_t array_size[] = { token.size/sizeof(double) };
                    hid_t array_space = H5Screate_simple(1,array_size,array_size);
                    hid_t array_type = H5Tarray_create(H5T_IEEE_F64LE,1,array_size);

                    QString data_name = QString::number(static_cast<unsigned long long>(token.time));

                    hid_t adata = H5Dcreate(file.adata,data_name.latin1(),array_type,array_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
                    H5Dwrite(adata,array_type,H5S_ALL,H5S_ALL,H5P_DEFAULT,data);

                    H5Dclose(adata);
                    H5Tclose(array_type);
                    H5Sclose(array_space);
                }

                fifo.writeDone();
            }

        } else if(token.type == OPEN) {

            if(state == RECORD)
                stopRecording(token.time);

            if(state != CLOSED)
                closeFile();

            QString filename = reinterpret_cast<char *>(fifo.read(token.size));
            fifo.readDone();

            if(openFile(filename))
                state = CLOSED;
            else
                state = OPENED;

        } else if(token.type == CLOSE) {

            if(state == RECORD)
                stopRecording(RT::OS::getTime());

            if(state != CLOSED)
                closeFile();

            state = CLOSED;

        } else if(token.type == START) {

            if(state == CLOSED) {
                QCustomEvent *event = new QCustomEvent(QNoFileOpenEvent);
                QApplication::postEvent(this,event);
            } else if(state == OPENED) {
                startRecording(token.time);
                state = RECORD;
            }

        } else if(token.type == STOP) {

            if(state == RECORD) {
                stopRecording(token.time);
                state = OPENED;
            }

        } else if(token.type == DONE) {

            if(state == RECORD)
                stopRecording(token.time,true);

            if(state != CLOSED)
                closeFile(true);

            break;
        } else if(token.type == PARAM) {
            param_change_t *data = reinterpret_cast<param_change_t *>(fifo.read(token.size));

            IO::Block *block = dynamic_cast<IO::Block *>(Settings::Manager::getInstance()->getObject(data->id));

            if(block && state == RECORD) {
                param_hdf_t param = {
                    data->step,
                    data->value,
                };

                hid_t param_type;
                param_type = H5Tcreate(H5T_COMPOUND,sizeof(param_hdf_t));
                H5Tinsert(param_type,"index",HOFFSET(param_hdf_t,index),H5T_STD_I64LE);
                H5Tinsert(param_type,"value",HOFFSET(param_hdf_t,value),H5T_IEEE_F64LE);

                QString parameter_name = QString::number(block->getID())+" "+block->getName()+" : "+block->getName(Workspace::PARAMETER,data->index);

                hid_t data = H5PTopen(file.pdata,parameter_name.latin1());
                H5PTappend(data,1,&param);
                H5PTclose(data);

                H5Tclose(param_type);
            }

            fifo.readDone();
        }

    }
}

int DataRecorder::Panel::openFile(QString &filename) {

#ifdef DEBUG
    if(!pthread_equal(pthread_self(),thread)) {
        ERROR_MSG("DataRecorder::Panel::openFile : called by invalid thread\n");
        PRINT_BACKTRACE();
    }
#endif

    if(QFile::exists(filename)) {
        QCustomEvent *event = new QCustomEvent(QFileExistsEvent);
        FileExistsEventData data;

        event->setData(&data);
        data.filename = filename;

        QApplication::postEvent(this,event);
        data.done.wait();

        if(data.response == 0)
            file.id = H5Fopen(filename.latin1(),H5F_ACC_RDWR,H5P_DEFAULT);
        else if(data.response == 1)
            file.id = H5Fcreate(filename.latin1(),H5F_ACC_TRUNC,H5P_DEFAULT,H5P_DEFAULT);
        else
            return -1;
    } else
        file.id = H5Fcreate(filename.latin1(),H5F_ACC_TRUNC,H5P_DEFAULT,H5P_DEFAULT);

    if(file.id < 0) {
        H5E_type_t error_type;
        size_t error_size;
        error_size = H5Eget_msg(file.id,&error_type,NULL,0);

        {
            char error_msg[error_size+1];
            H5Eget_msg(file.id,&error_type,error_msg,error_size);
            error_msg[error_size] = 0;
            H5Eclear(file.id);

            ERROR_MSG("DataRecorder::Panel::processData : failed to open \"%s\" for writing with error : %s\n",filename.latin1(),error_msg);
            return -1;
        }
    }

    QCustomEvent *event = new QCustomEvent(QSetFileNameEditEvent);
    SetFileNameEditEventData data;

    event->setData(&data);
    data.filename = filename;

    QApplication::postEvent(this,event);
    data.done.wait();

    return 0;
}

void DataRecorder::Panel::closeFile(bool shutdown) {

#ifdef DEBUG
    if(!pthread_equal(pthread_self(),thread)) {
        ERROR_MSG("DataRecorder::Panel::closeFile : called by invalid thread\n");
        PRINT_BACKTRACE();
    }
#endif

    H5Fclose(file.id);

    if(!shutdown) {
        QCustomEvent *event = new QCustomEvent(QSetFileNameEditEvent);
        SetFileNameEditEventData data;

        event->setData(&data);
        data.filename = "";

        QApplication::postEvent(this,event);
        data.done.wait();
    }
}

int DataRecorder::Panel::startRecording(long long timestamp) {

#ifdef DEBUG
    if(!pthread_equal(pthread_self(),thread)) {
        ERROR_MSG("DataRecorder::Panel::startRecording : called by invalid thread\n");
        PRINT_BACKTRACE();
    }
#endif

    size_t trial_num;
    QString trial_name;

    H5Eset_auto(H5E_DEFAULT,NULL,NULL);

    for(trial_num=1;;++trial_num) {
        trial_name = "/Trial"+QString::number(trial_num);
        file.trial = H5Gopen(file.id,trial_name.latin1(),H5P_DEFAULT);

        if(file.trial < 0) {
            H5Eclear(H5E_DEFAULT);
            break;
        } else
            H5Gclose(file.trial);
    }

    file.trial = H5Gcreate(file.id,trial_name.latin1(),H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    file.pdata = H5Gcreate(file.trial,"Parameters",H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    file.adata = H5Gcreate(file.trial,"Asyncronous Data",H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    file.sdata = H5Gcreate(file.trial,"Syncronous Data",H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);

    hid_t scalar_space = H5Screate(H5S_SCALAR);
    hid_t string_type = H5Tcopy(H5T_C_S1);
    size_t string_size = 512;
    H5Tset_size(string_type,string_size);
    hid_t data;

    long long period = RT::System::getInstance()->getPeriod();
    data = H5Dcreate(file.trial,"Period (ns)",H5T_STD_U64LE,scalar_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    H5Dwrite(data,H5T_STD_U64LE,H5S_ALL,H5S_ALL,H5P_DEFAULT,&period);
    H5Dclose(data);

    long long downsample = downsample_rate;
    data = H5Dcreate(file.trial,"Downsampling Rate",H5T_STD_U64LE,scalar_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    H5Dwrite(data,H5T_STD_U64LE,H5S_ALL,H5S_ALL,H5P_DEFAULT,&downsample);
    H5Dclose(data);

    data = H5Dcreate(file.trial,"Timestamp Start (ns)",H5T_STD_U64LE,scalar_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    H5Dwrite(data,H5T_STD_U64LE,H5S_ALL,H5S_ALL,H5P_DEFAULT,&timestamp);
    H5Dclose(data);

    data = H5Dcreate(file.trial,"Date",string_type,scalar_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    H5Dwrite(data,string_type,H5S_ALL,H5S_ALL,H5P_DEFAULT,QDateTime::currentDateTime().toString(Qt::ISODate).latin1());
    H5Dclose(data);

    hid_t param_type;
    param_type = H5Tcreate(H5T_COMPOUND,sizeof(param_hdf_t));
    H5Tinsert(param_type,"index",HOFFSET(param_hdf_t,index),H5T_STD_I64LE);
    H5Tinsert(param_type,"value",HOFFSET(param_hdf_t,value),H5T_IEEE_F64LE);

    for(RT::List<Channel>::iterator i = channels.begin(), end = channels.end();i != end;++i) {
        IO::Block *block = i->block;
        for(size_t j = 0;j < block->getCount(Workspace::PARAMETER);++j) {
            QString parameter_name = QString::number(block->getID())+" "+block->getName()+" : "+block->getName(Workspace::PARAMETER,j);
            data = H5PTcreate_fl(file.pdata,parameter_name.latin1(),param_type,sizeof(param_hdf_t),-1);
            struct param_hdf_t value = {
                0,
                block->getValue(Workspace::PARAMETER,j),
            };
            H5PTappend(data,1,&value);
            H5PTclose(data);
        }
        for(size_t j = 0;j < block->getCount(Workspace::COMMENT);++j) {
            QString comment_name = QString::number(block->getID())+" "+block->getName()+" : "+block->getName(Workspace::COMMENT,j);
            hsize_t dims = dynamic_cast<Workspace::Instance *>(block)->getValueString(Workspace::COMMENT,j).size()+1;
            hid_t comment_space = H5Screate_simple(1,&dims,&dims);
            data = H5Dcreate(file.pdata,comment_name.latin1(),H5T_C_S1,comment_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
            H5Dwrite(data,H5T_C_S1,H5S_ALL,H5S_ALL,H5P_DEFAULT,dynamic_cast<Workspace::Instance *>(block)->getValueString(Workspace::COMMENT,j).c_str());
            H5Dclose(data);
        }
    }

    H5Tclose(param_type);

    size_t count = 0;
    for(RT::List<Channel>::iterator i = channels.begin(), end = channels.end();i != end;++i) {
        QString channel_name = "Channel " + QString::number(++count) + " Name";
        hid_t data = H5Dcreate(file.sdata,channel_name.latin1(),string_type,scalar_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
        H5Dwrite(data,string_type,H5S_ALL,H5S_ALL,H5P_DEFAULT,i->name.latin1());
        H5Dclose(data);
    }

    H5Tclose(string_type);
    H5Sclose(scalar_space);

    if(channels.size()) {
        hsize_t array_size[] = { channels.size() };
        hid_t array_type = H5Tarray_create(H5T_IEEE_F64LE,1,array_size);
        file.cdata = H5PTcreate_fl(file.sdata,"Channel Data",array_type,(hsize_t)64,1);
        H5Tclose(array_type);
    }

    file.idx = 0;

    QCustomEvent *event = new QCustomEvent(QDisableGroupsEvent);
    QApplication::postEvent(this,event);

    return 0;
}

void DataRecorder::Panel::stopRecording(long long timestamp,bool shutdown) {

#ifdef DEBUG
    if(!pthread_equal(pthread_self(),thread)) {
        ERROR_MSG("DataRecorder::Panel::stopRecording : called by invalid thread\n");
        PRINT_BACKTRACE();
    }
#endif

    hid_t scalar_space = H5Screate(H5S_SCALAR);
    hid_t data = H5Dcreate(file.trial,"Timestamp Stop (ns)",H5T_STD_U64LE,scalar_space,H5P_DEFAULT,H5P_DEFAULT,H5P_DEFAULT);
    H5Dwrite(data,H5T_STD_U64LE,H5S_ALL,H5S_ALL,H5P_DEFAULT,&timestamp);
    H5Dclose(data);
    H5Sclose(scalar_space);

    H5PTclose(file.cdata);
    H5Gclose(file.sdata);
    H5Gclose(file.pdata);
    H5Gclose(file.adata);
    H5Gclose(file.trial);

    H5Fflush(file.id,H5F_SCOPE_LOCAL);
    void *file_handle;
    H5Fget_vfd_handle(file.id,H5P_DEFAULT,&file_handle);
    if(fsync(*static_cast<int *>(file_handle))) {
        DEBUG_MSG("DataRecorder::Panel::stopRecording : fsync failed, running sync\n");
        sync();
    }

    if(!shutdown) {
        QCustomEvent *event = new QCustomEvent(QEnableGroupsEvent);
        QApplication::postEvent(this,event);
    }
}

extern "C" Plugin::Object *createRTXIPlugin(void *) {
    return DataRecorder::Plugin::getInstance();
}

DataRecorder::Plugin::Plugin(void) {
    menuID = MainWindow::getInstance()->createControlMenuItem("Data Recorder",this,SLOT(createDataRecorderPanel(void)));
}

DataRecorder::Plugin::~Plugin(void) {
    MainWindow::getInstance()->removeControlMenuItem(menuID);
    while(panelList.size())
        delete panelList.front();
    instance = 0;
}

DataRecorder::Panel *DataRecorder::Plugin::createDataRecorderPanel(void) {
    Panel *panel = new Panel(MainWindow::getInstance()->centralWidget());
    panelList.push_back(panel);
    return panel;
}

void DataRecorder::Plugin::removeDataRecorderPanel(DataRecorder::Panel *panel) {
    panelList.remove(panel);
}

void DataRecorder::Plugin::doDeferred(const Settings::Object::State &s) {
    size_t i = 0;
    for(std::list<Panel *>::iterator j = panelList.begin(),end = panelList.end();j != end;++j)
        (*j)->deferred(s.loadState(QString::number(i++)));
}

void DataRecorder::Plugin::doLoad(const Settings::Object::State &s) {
    for(size_t i=0;i < static_cast<size_t>(s.loadInteger("Num Panels"));++i) {
        Panel *panel = new Panel(MainWindow::getInstance()->centralWidget());
        panelList.push_back(panel);
        panel->load(s.loadState(QString::number(i)));
    }
}

void DataRecorder::Plugin::doSave(Settings::Object::State &s) const {
    s.saveInteger("Num Panels",panelList.size());
    size_t n = 0;
    for(std::list<Panel *>::const_iterator i = panelList.begin(),end = panelList.end();i != end;++i)
        s.saveState(QString::number(n++),(*i)->save());
}

static Mutex mutex;
DataRecorder::Plugin *DataRecorder::Plugin::instance = 0;

DataRecorder::Plugin *DataRecorder::Plugin::getInstance(void) {
    if(instance)
        return instance;

    /*************************************************************************
     * Seems like alot of hoops to jump through, but allocation isn't        *
     *   thread-safe. So effort must be taken to ensure mutual exclusion.    *
     *************************************************************************/

    Mutex::Locker lock(&::mutex);
    if(!instance)
        instance = new Plugin();

    return instance;
}
