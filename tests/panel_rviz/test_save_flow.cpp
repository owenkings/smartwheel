#include <QTest>
#include <QTimer>
#include <QTemporaryDir>
#define WC_SAVE_CHOICE_TEST
#include "../../src/wc_bringup/src/map_save_choice.cpp"

class SaveFlowTest : public QObject {
  Q_OBJECT
private slots:
  void path_dialog_creates_one_usable_browser_after_native_layout_initialization() {
    SavePathInput input("synthetic");
    input.setInputMode(QInputDialog::TextInput);
    input.setTextValue("/unused/new_map");
    input.show();
    QTRY_VERIFY(input.isVisible());
    QCOMPARE(input.okButtonText(), QString("确认保存"));
    QCOMPARE(input.cancelButtonText(), QString("返回上一步"));
    QVERIFY(input.layout());
    auto buttons = input.findChildren<QPushButton *>("map_save_browse_button");
    QCOMPARE(buttons.size(), 1);
    QVERIFY(buttons.front()->isVisible());
    input.hide();
    input.show();
    QApplication::processEvents();
    QCOMPARE(input.findChildren<QPushButton *>("map_save_browse_button").size(), 1);
    QCOMPARE(input.textValue(), QString("/unused/new_map"));
    input.close();
  }
  void cancel_path_returns_to_decision_without_deleting() {
    QTemporaryDir root; QVERIFY(root.isValid());
    int choice_count = 0, path_count = 0;
    QTimer timer;
    connect(&timer, &QTimer::timeout, [&]() {
      if (auto * box = qobject_cast<QMessageBox *>(QApplication::activeModalWidget())) {
        ++choice_count;
        for (auto * button : box->buttons()) {
          const auto role = choice_count == 1 ? QMessageBox::AcceptRole : QMessageBox::RejectRole;
          if (box->buttonRole(button) == role) {button->click(); break;}
        }
      } else if (auto * input = qobject_cast<QInputDialog *>(QApplication::activeModalWidget())) {
        QVERIFY(input->layout());
        QCOMPARE(input->findChildren<QPushButton *>("map_save_browse_button").size(), 1);
        ++path_count; input->reject();
      }
    });
    timer.start(10);
    const auto result = ask_map_save_choice("synthetic", root.path() + "/new_map");
    QCOMPARE(result.value("decision").toString(), QString("pending"));
    QCOMPARE(choice_count, 2); QCOMPARE(path_count, 1);
    QVERIFY(QDir(root.path()).entryList(QDir::NoDotAndDotDot | QDir::AllEntries).isEmpty());
  }
  void repeated_path_cancellation_then_save_preserves_destination_and_creates_no_files() {
    QTemporaryDir root; QVERIFY(root.isValid());
    const QString destination = root.path() + "/chosen_map";
    int choices = 0, paths = 0;
    QTimer timer;
    connect(&timer, &QTimer::timeout, [&]() {
      if (auto * box = qobject_cast<QMessageBox *>(QApplication::activeModalWidget())) {
        ++choices;
        for (auto * button : box->buttons()) {
          if (box->buttonRole(button) == QMessageBox::AcceptRole) {button->click(); break;}
        }
      } else if (auto * input = qobject_cast<QInputDialog *>(QApplication::activeModalWidget())) {
        ++paths;
        if (paths < 3) {input->reject();}
        else {input->setTextValue(destination); input->accept();}
      }
    });
    timer.start(10);
    const auto result = ask_map_save_choice("synthetic", root.path() + "/default_map");
    QCOMPARE(result.value("decision").toString(), QString("save"));
    QCOMPARE(result.value("destination").toString(), destination);
    QCOMPARE(choices, 3); QCOMPARE(paths, 3);
    QVERIFY(QDir(root.path()).entryList(QDir::NoDotAndDotDot | QDir::AllEntries).isEmpty());
  }
};
QTEST_MAIN(SaveFlowTest)
#include "test_save_flow.moc"
