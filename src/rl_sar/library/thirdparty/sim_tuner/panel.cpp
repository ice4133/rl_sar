// SPDX-License-Identifier: Apache-2.0
#include "parameters.hpp"
#include <QtWidgets>
#include <QTcpSocket>
#include <QHostAddress>
#include <algorithm>
#include <cmath>
#include <deque>

namespace {
using namespace sim_tuner;
struct Sample { double t, q, target_q, dq, target_dq, tau; };

class Plot : public QWidget {
public:
    explicit Plot(QWidget* parent = nullptr) : QWidget(parent) { setMinimumHeight(270); }
    void add(Sample sample) {
        if (!data_.empty() && sample.t < data_.back().t) data_.clear();
        if (!data_.empty() && sample.t == data_.back().t) return;
        data_.push_back(sample);
        while (data_.size() > 1500 || (!data_.empty() && sample.t - data_.front().t > 10)) data_.pop_front();
        update();
    }
    void clear() { data_.clear(); update(); }
protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this); p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), QColor("#111b2a"));
        const QStringList labels{"关节位置  rad", "关节速度  rad/s", "实际力矩  N·m"};
        for (int plot = 0; plot < 3; ++plot) {
            QRectF area(62, 30 + plot * (height()-35)/3.0, width()-80, (height()-35)/3.0-30);
            p.setPen(QColor("#9badc5")); p.drawText(QPointF(12, area.top()-8), labels[plot]);
            p.setPen(QColor("#29364b")); p.drawRect(area);
            if (data_.empty()) continue;
            double low = 0, high = 0;
            auto value = [plot](const Sample& s, bool target) {
                return plot == 0 ? (target ? s.target_q : s.q) : plot == 1 ? (target ? s.target_dq : s.dq) : s.tau;
            };
            for (const auto& s : data_) for (bool target : {false, true}) {
                low = std::min(low, value(s, target)); high = std::max(high, value(s, target));
            }
            const double margin = std::max(.01, (high-low)*.12); low -= margin; high += margin;
            p.setPen(QColor("#9badc5"));
            p.drawText(QRectF(0, area.top()-8, 56, 20), Qt::AlignRight, QString::number(high, 'g', 3));
            p.drawText(QRectF(0, area.bottom()-12, 56, 20), Qt::AlignRight, QString::number(low, 'g', 3));
            for (bool target : {true, false}) {
                if (plot == 2 && target) continue;
                QPainterPath path;
                bool first = true;
                for (const auto& s : data_) {
                    double x = area.right() - (data_.back().t-s.t)/10*area.width();
                    double y = area.bottom() - (value(s, target)-low)/(high-low)*area.height();
                    if (first) path.moveTo(x,y); else path.lineTo(x,y);
                    first = false;
                }
                p.setPen(QPen(QColor(target ? "#f6bd60" : "#4ed6b2"), 1.7)); p.drawPath(path);
            }
        }
        p.setPen(QColor("#9badc5"));
        p.drawText(QRect(0, height()-20, width()-12, 20), Qt::AlignRight,
                   "最近 10 秒仿真时间 · 绿色：实际值  黄色：延迟后采用的目标 · 曲线采样 10 Hz");
    }
private:
    std::deque<Sample> data_;
};

class Panel : public QMainWindow {
public:
    explicit Panel(int port) {
        setWindowTitle("rl_sar · MuJoCo 统一调参面板"); resize(1260, 850);
        auto* root = new QWidget; setCentralWidget(root);
        auto* layout = new QVBoxLayout(root);
        auto* connection = new QHBoxLayout;
        auto* title = new QLabel("MuJoCo  /  Sim-to-real 调参");
        title->setStyleSheet("font-size:20px;font-weight:600;"); connection->addWidget(title); connection->addStretch();
        connection->addWidget(new QLabel("127.0.0.1 :"));
        port_ = new QSpinBox; port_->setRange(1,65535); port_->setValue(port); connection->addWidget(port_);
        connect_ = new QPushButton("连接仿真"); connection->addWidget(connect_); layout->addLayout(connection);
        state_ = new QLabel("未连接。仿真启动参数追加 --tuner-port 9870，再点击连接。");
        state_->setWordWrap(true); layout->addWidget(state_);
        auto* splitter = new QSplitter;
        auto* left = new QWidget; auto* leftLayout = new QVBoxLayout(left); leftLayout->setContentsMargins(0,0,0,0);
        auto* tabs = new QTabWidget;
        for (const QString& group : {QString("接触"),QString("驱动"),QString("观测"),QString("模型")}) {
            auto* page = new QWidget; auto* grid = new QGridLayout(page);
            grid->addWidget(new QLabel("参数 / 单位"),0,0); grid->addWidget(new QLabel("设置值"),0,1);
            grid->addWidget(new QLabel("已生效"),0,3);
            int row = 1;
            for (const auto& spec : kParameters) {
                if (group != QString::fromUtf8(spec.group)) continue;
                const QString key = spec.key;
                auto* label = new QLabel(QString::fromUtf8(spec.label) + (spec.reset ? " *" : ""));
                label->setToolTip(QString::fromUtf8(spec.help)); grid->addWidget(label,row,0);
                auto* spin = new QDoubleSpinBox;
                spin->setDecimals(spec.step < .01 ? 3 : spec.step < 1 ? 2 : 0);
                spin->setRange(spec.minimum,spec.maximum); spin->setSingleStep(spec.step); spin->setValue(spec.initial);
                spin->setSuffix(QString(" ") + spec.unit); spin->setToolTip(QString::fromUtf8(spec.help));
                spin->setKeyboardTracking(false); spin->setMinimumWidth(130); grid->addWidget(spin,row,1);
                auto* slider = new QSlider(Qt::Horizontal);
                slider->setRange(0,qRound((spec.maximum-spec.minimum)/spec.step));
                slider->setValue(qRound((spec.initial-spec.minimum)/spec.step)); slider->setMinimumWidth(90);
                grid->addWidget(slider,row,2);
                auto* applied = new QLabel("—"); applied->setMinimumWidth(65); grid->addWidget(applied,row,3);
                editors_[key] = spin; applied_[key] = applied;
                const double minimum = spec.minimum, step = spec.step;
                QObject::connect(slider,&QSlider::valueChanged,this,[spin,minimum,step](int v) { spin->setValue(minimum+v*step); });
                QObject::connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,
                    [this,slider,key,minimum,step](double v) {
                        QSignalBlocker blocked(slider); slider->setValue(qRound((v-minimum)/step));
                        if (!loading_) {
                            markDrafts();
                            if (live_->isChecked() && !Spec(key.toStdString()).reset) debounce_.start(150);
                        }
                    });
                ++row;
            }
            grid->setRowStretch(row,1);
            auto* scroll = new QScrollArea; scroll->setWidgetResizable(true); scroll->setWidget(page);
            tabs->addTab(scroll,group);
        }
        leftLayout->addWidget(tabs);
        live_ = new QCheckBox("实时应用普通参数（拖动停止后发送）"); live_->setChecked(true); leftLayout->addWidget(live_);
        auto* help = new QLabel("* 模型参数需“应用并复位”。摩擦覆盖关闭时保持 XML 原值。\n倍率相对启动基准计算；窗口关闭后仿真保留当前参数。");
        help->setWordWrap(true); help->setStyleSheet("color:#9badc5;"); leftLayout->addWidget(help);
        auto* applyRow = new QHBoxLayout;
        auto* apply = new QPushButton("应用普通参数"); auto* reset = new QPushButton("应用全部并复位");
        applyRow->addWidget(apply); applyRow->addWidget(reset); leftLayout->addLayout(applyRow);
        auto* fileRow = new QHBoxLayout;
        auto* save = new QPushButton("保存设置…"); auto* load = new QPushButton("加载设置…");
        auto* defaults = new QPushButton("恢复基准并复位");
        fileRow->addWidget(save); fileRow->addWidget(load); fileRow->addWidget(defaults); leftLayout->addLayout(fileRow);
        splitter->addWidget(left);
        auto* right = new QWidget; auto* rightLayout = new QVBoxLayout(right);
        summary_ = new QLabel("等待仿真状态…"); summary_->setWordWrap(true); summary_->setMinimumHeight(95); rightLayout->addWidget(summary_);
        joint_ = new QComboBox; rightLayout->addWidget(joint_);
        plot_ = new Plot; rightLayout->addWidget(plot_,1);
        log_ = new QPlainTextEdit; log_->setReadOnly(true); log_->setMaximumBlockCount(200); log_->setMaximumHeight(150); rightLayout->addWidget(log_);
        splitter->addWidget(right); splitter->setSizes({640,580}); layout->addWidget(splitter,1);
        statusBar()->showMessage("所有数值在仿真端校验；模型原始 XML 和策略 YAML 不会被覆盖。");

        debounce_.setSingleShot(true);
        QObject::connect(&debounce_,&QTimer::timeout,this,[this] { applyChanges(false); });
        QObject::connect(apply,&QPushButton::clicked,this,[this] { applyChanges(false); });
        QObject::connect(reset,&QPushButton::clicked,this,[this] { applyChanges(true); });
        QObject::connect(save,&QPushButton::clicked,this,[this] { saveProfile(); });
        QObject::connect(load,&QPushButton::clicked,this,[this] { loadProfile(); });
        QObject::connect(defaults,&QPushButton::clicked,this,[this] { send("defaults",{},true); });
        QObject::connect(joint_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this] { plot_->clear(); });
        QObject::connect(connect_,&QPushButton::clicked,this,[this] {
            if (socket_.state() != QAbstractSocket::UnconnectedState) socket_.abort();
            else { connect_->setText("取消连接"); socket_.connectToHost(QHostAddress::LocalHost,port_->value()); }
        });
        QObject::connect(&socket_,&QTcpSocket::connected,this,[this] {
            initialize_ = true; incoming_.clear(); pending_.clear(); lastReply_.restart();
            connect_->setText("断开连接"); port_->setEnabled(false); append("已连接，读取仿真当前参数。"); send("status");
        });
        QObject::connect(&socket_,&QTcpSocket::disconnected,this,[this] { disconnected("连接已断开"); });
        QObject::connect(&socket_,QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),this,
                         [this](QAbstractSocket::SocketError) { disconnected(socket_.errorString()); });
        QObject::connect(&socket_,&QTcpSocket::readyRead,this,[this] { readReplies(); });
        QObject::connect(&poll_,&QTimer::timeout,this,[this] {
            if (socket_.state() != QAbstractSocket::ConnectedState) return;
            if (lastReply_.elapsed() > 3000) { socket_.abort(); append("仿真 3 秒未响应，请检查仿真状态后重新连接。"); return; }
            if (pending_.isEmpty()) send("status");
        });
        poll_.start(100);
        setStyleSheet("QMainWindow,QWidget{background:#172337;color:#e5edf7;font-size:13px;}"
                      "QLineEdit,QAbstractSpinBox,QPlainTextEdit,QComboBox{background:#101b2b;border:1px solid #394c68;border-radius:4px;padding:5px;}"
                      "QPushButton{background:#2d4868;border:1px solid #456383;border-radius:5px;padding:8px;}"
                      "QPushButton:hover{background:#396086;} QTabBar::tab{padding:10px 18px;}"
                      "QTabBar::tab:selected{background:#2d4868;} QSlider::groove:horizontal{height:4px;background:#3b4d65;}"
                      "QSlider::handle:horizontal{background:#4ed6b2;width:12px;margin:-5px 0;border-radius:5px;}");
    }
    // Also used by the offscreen UI smoke check.
    ~Panel() override {
        poll_.stop(); debounce_.stop();
        // Socket destruction can emit disconnected; stop callbacks before members die.
        socket_.disconnect(this); socket_.abort();
    }
    void autoConnect() { connect_->click(); }
    bool connected() const { return !initialize_ && socket_.state() == QAbstractSocket::ConnectedState; }
private:
    void append(const QString& message) { log_->appendPlainText(QTime::currentTime().toString("HH:mm:ss ")+message); }
    QJsonObject draft() const {
        QJsonObject values; for (auto it = editors_.cbegin(); it != editors_.cend(); ++it) values[it.key()] = it.value()->value();
        return values;
    }
    void markDrafts() {
        for (auto it = editors_.cbegin(); it != editors_.cend(); ++it) {
            const bool differs = !actual_.contains(it.key()) || std::abs(actual_[it.key()].toDouble()-it.value()->value()) > 1e-8;
            it.value()->setStyleSheet(differs ? "border:1px solid #f6bd60;" : "");
        }
    }
    void send(const QString& operation, QJsonObject values = {}, bool reset = false) {
        if (socket_.state() != QAbstractSocket::ConnectedState) {
            if (operation != "status") append("未连接，设置尚未发送。"); return;
        }
        if (pending_.size() >= 32) { append("等待仿真处理已有请求。"); return; }
        const int id = ++request_;
        QJsonObject message{{"protocol",kProtocol},{"id",id},{"op",operation},{"reset",reset}};
        if (operation == "set") message["params"] = values;
        pending_[id] = operation;
        socket_.write(QJsonDocument(message).toJson(QJsonDocument::Compact)+'\n');
    }
    void applyChanges(bool reset) {
        debounce_.stop();
        if (initialize_ || socket_.state() != QAbstractSocket::ConnectedState) { append("连接并读取仿真参数后可应用。"); return; }
        QJsonObject patch;
        for (auto it = editors_.cbegin(); it != editors_.cend(); ++it) {
            if (!reset && Spec(it.key().toStdString()).reset) continue;
            if (!actual_.contains(it.key()) || std::abs(actual_[it.key()].toDouble()-it.value()->value()) > 1e-8)
                patch[it.key()] = it.value()->value();
        }
        if (!patch.isEmpty() || reset) send("set",patch,reset);
    }
    void readReplies() {
        incoming_ += socket_.readAll();
        if (incoming_.size() > 1048576) { socket_.abort(); append("响应过大，已断开。"); return; }
        int newline;
        while ((newline = incoming_.indexOf('\n')) >= 0) {
            QByteArray line = incoming_.left(newline); incoming_.remove(0,newline+1);
            QJsonParseError error; auto doc = QJsonDocument::fromJson(line,&error);
            if (error.error != QJsonParseError::NoError || !doc.isObject()) {
                socket_.abort(); append("收到无效 JSON 响应。"); return;
            }
            auto data = doc.object();
            if (data["protocol"].toInt() != kProtocol) { socket_.abort(); append("协议版本不一致。"); return; }
            const QString operation = pending_.take(data["id"].toInt()); lastReply_.restart();
            if (!data["ok"].toBool()) { append("仿真拒绝修改："+data["error"].toString()); continue; }
            actual_ = data["params"].toObject();
            if (initialize_ || operation == "defaults") {
                loading_ = true;
                for (auto it = editors_.begin(); it != editors_.end(); ++it)
                    if (actual_.contains(it.key())) it.value()->setValue(actual_[it.key()].toDouble());
                loading_ = false; initialize_ = false; plot_->clear();
            }
            for (auto it = applied_.begin(); it != applied_.end(); ++it)
                it.value()->setText(QString::number(actual_[it.key()].toDouble(),'g',6));
            markDrafts();
            state_->setText(QString("已连接 · %1 · %2 驱动 · %3 · 参数版本 %4 · 生效于 t=%5 s")
                            .arg(data["robot"].toString(),data["drive"].toString(),data["running"].toBool()?"运行中":"已暂停")
                            .arg(data["version"].toDouble(),0,'f',0).arg(data["applied_at"].toDouble(),0,'f',3));
            summary_->setText(QString("仿真时间  %1 s       物理步长  %2 ms\n机器人质量  %3 kg       基座高度  %4 m\n滑动摩擦范围  %5 ～ %6       等效延迟  %7 ms\n在途指令  %8       延迟精度以物理步长为限")
                .arg(data["sim_time"].toDouble(),0,'f',3).arg(data["timestep_ms"].toDouble(),0,'g',5)
                .arg(data["mass"].toDouble(),0,'f',3).arg(data["height"].toDouble(),0,'f',3)
                .arg(data["friction_min"].toDouble(),0,'g',4).arg(data["friction_max"].toDouble(),0,'g',4)
                .arg(actual_["delay_ms"].toDouble(),0,'g',5).arg(data["queued"].toInt()));
            QStringList names; for (auto name : data["joint_names"].toArray()) names << name.toString();
            QStringList current; for (int i = 0; i < joint_->count(); ++i) current << joint_->itemText(i);
            if (names != current) { joint_->clear(); joint_->addItems(names); }
            const int j = joint_->currentIndex();
            auto at = [&data,j](const char* key) { return data[key].toArray().at(j).toDouble(); };
            if (j >= 0 && j < data["q"].toArray().size())
                plot_->add({data["sim_time"].toDouble(),at("q"),at("target_q"),at("dq"),at("target_dq"),at("tau")});
            if (operation != "status") append(QString("参数已生效，版本 %1。").arg(data["version"].toDouble(),0,'f',0));
        }
    }
    void disconnected(const QString& reason) {
        debounce_.stop(); pending_.clear(); incoming_.clear(); initialize_ = true; actual_ = {};
        connect_->setText("连接仿真"); port_->setEnabled(true); state_->setText("未连接 · "+reason);
        for (auto* label : applied_) label->setText("—"); append(reason); markDrafts();
    }
    void saveProfile() {
        const QString file = QFileDialog::getSaveFileName(this,"保存面板设置", "sim_tuner.json", "JSON (*.json)");
        if (file.isEmpty()) return;
        QSaveFile output(file);
        QJsonObject profile{{"protocol",kProtocol},{"params",draft()}};
        if (!output.open(QIODevice::WriteOnly) || output.write(QJsonDocument(profile).toJson()) < 0 || !output.commit()) {
            append("保存失败："+output.errorString()); return;
        }
        append("已保存设置："+file);
    }
    void loadProfile() {
        const QString file = QFileDialog::getOpenFileName(this,"加载面板设置",{},"JSON (*.json)");
        if (file.isEmpty()) return;
        QFile input(file); if (!input.open(QIODevice::ReadOnly)) { append("读取失败："+input.errorString()); return; }
        if (input.size() > 65536) { append("配置文件过大。"); return; }
        QJsonParseError parse; auto doc = QJsonDocument::fromJson(input.readAll(),&parse);
        if (parse.error != QJsonParseError::NoError || !doc.isObject() || doc["protocol"].toInt() != kProtocol || !doc["params"].isObject()) {
            append("配置格式或版本错误。"); return;
        }
        auto values = doc["params"].toObject();
        try {
            for (auto it = values.begin(); it != values.end(); ++it) {
                if (!it.value().isDouble()) throw std::runtime_error("参数必须为数值");
                Validate(it.key().toStdString(),it.value().toDouble());
            }
        } catch (const std::exception& e) { append(QString("配置未加载：")+e.what()); return; }
        loading_ = true;
        for (auto it = values.begin(); it != values.end(); ++it) editors_[it.key()]->setValue(it.value().toDouble());
        loading_ = false; markDrafts();
        append("设置已加载到面板，点击应用按钮发送；模型参数使用“应用全部并复位”。");
    }
    QTcpSocket socket_;
    QTimer poll_, debounce_;
    QElapsedTimer lastReply_;
    QByteArray incoming_;
    QMap<int,QString> pending_;
    QMap<QString,QDoubleSpinBox*> editors_;
    QMap<QString,QLabel*> applied_;
    QJsonObject actual_;
    QSpinBox* port_;
    QPushButton* connect_;
    QLabel *state_, *summary_;
    QCheckBox* live_;
    QComboBox* joint_;
    QPlainTextEdit* log_;
    Plot* plot_;
    int request_ = 0;
    bool loading_ = true, initialize_ = true;
public:
    void ready() { loading_ = false; }
};
}  // namespace

int main(int argc, char** argv) {
    QApplication app(argc,argv);
    QCommandLineParser parser; parser.setApplicationDescription("MuJoCo C++ 统一调参面板"); parser.addHelpOption();
    parser.addOption({"port","本机仿真调参端口","port",QString::number(sim_tuner::kDefaultPort)});
    parser.addOption({"connect","启动后自动连接"});
    parser.addOption({"smoke-test","离屏界面检查，短暂运行后退出"});
    parser.addOption({"screenshot","保存面板截图","path"});
    parser.process(app);
    bool ok; const int port = parser.value("port").toInt(&ok);
    if (!ok || port < 1 || port > 65535) { qCritical("Invalid port"); return 2; }
    Panel panel(port); panel.ready(); panel.show();
    if (parser.isSet("connect")) QTimer::singleShot(0,&panel,[&panel] { panel.autoConnect(); });
    if (parser.isSet("screenshot")) QTimer::singleShot(600,&panel,[&panel,&parser] {
        if (!panel.grab().save(parser.value("screenshot"))) qWarning("Cannot save screenshot");
    });
    if (parser.isSet("smoke-test")) QTimer::singleShot(900,&app,[&app,&panel,&parser] {
        app.exit(parser.isSet("connect") && !panel.connected() ? 3 : 0);
    });
    return app.exec();
}
