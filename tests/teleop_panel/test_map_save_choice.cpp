#include <QTest>
#include <QTimer>
#include <QTemporaryDir>
#define WC_SAVE_CHOICE_TEST
#include "../../src/wc_bringup/src/map_save_choice.cpp"

class SaveChoiceTest : public QObject {
  Q_OBJECT
private slots:
  void explicit_no_discards() {
    QTimer::singleShot(10, []() {
      auto * box=qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
      QVERIFY(box);
      for (auto * button:box->buttons()) {
        if (box->buttonRole(button)==QMessageBox::DestructiveRole) {button->click(); return;}
      }
      QFAIL("discard choice missing");
    });
    const auto result=ask_map_save_choice("synthetic", "/unused/new_map");
    QCOMPARE(result.value("decision").toString(), QString("discard"));
  }
  void closing_is_pending() {
    QTimer::singleShot(10, []() {
      auto * box=qobject_cast<QMessageBox *>(QApplication::activeModalWidget());
      QVERIFY(box); box->close();
    });
    QCOMPARE(ask_map_save_choice("synthetic", "/unused/new_map").value("decision").toString(),QString("pending"));
  }
  void affirmative_choice_selects_new_path_without_writing() {
    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString target=directory.path()+"/new_map";
    QTimer timer;
    connect(&timer,&QTimer::timeout,[&]() {
      if (auto * box=qobject_cast<QMessageBox *>(QApplication::activeModalWidget())) {
        for(auto * button:box->buttons()) {if(box->buttonRole(button)==QMessageBox::AcceptRole) {button->click();break;}}
      } else if (auto * input=qobject_cast<QInputDialog *>(QApplication::activeModalWidget())) {
        input->setTextValue(target); input->accept();
      }
    });
    timer.start(10);
    const auto result=ask_map_save_choice("synthetic",target);
    QCOMPARE(result.value("decision").toString(),QString("save"));
    QCOMPARE(result.value("destination").toString(),target);
    QVERIFY(!QFileInfo(target).exists());
  }
};
QTEST_MAIN(SaveChoiceTest)
#include "test_map_save_choice.moc"
