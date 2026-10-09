// Test-only driver of the production Qt dialogs. Never installed or used by hardware.
#define WC_SAVE_CHOICE_TEST
#include "../../src/wc_bringup/src/map_save_choice.cpp"
#include <QTimer>
#include <QScreen>
#include <cstdlib>

int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  if (argc < 3) {return 2;}
  const QString action = QString::fromLocal8Bit(argv[1]);
  const QString destination = QString::fromLocal8Bit(argv[2]);
  const QString screenshots = argc > 3 ? QString::fromLocal8Bit(argv[3]) : QString();
  int choices = 0, paths = 0;
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]() {
    QWidget * modal = QApplication::activeModalWidget();
    if (auto * box = qobject_cast<QMessageBox *>(modal)) {
      ++choices;
      if (!screenshots.isEmpty()) {box->grab().save(screenshots + "/save_question.png");}
      auto role = action == "discard" ? QMessageBox::DestructiveRole :
        action == "pending" || (action == "cancel_then_pending" && choices > 1)
        ? QMessageBox::RejectRole : QMessageBox::AcceptRole;
      for (auto * button : box->buttons()) {
        if (box->buttonRole(button) == role) {button->click(); break;}
      }
    } else if (auto * input = qobject_cast<QInputDialog *>(modal)) {
      ++paths;
      if (!screenshots.isEmpty()) {input->grab().save(screenshots + "/save_path.png");}
      if ((action == "cancel_then_save" || action == "cancel_then_pending") && paths == 1) {
        input->reject();
      } else {input->setTextValue(destination); input->accept();}
    }
  });
  timer.start(150);
  QTimer::singleShot(8000, &app, []() {std::exit(3);});
  auto result = ask_map_save_choice("SYNTHETIC_ACCEPTANCE", destination);
  result.insert("test_choice_count", choices);
  result.insert("test_path_count", paths);
  QTextStream(stdout) << QJsonDocument(result).toJson(QJsonDocument::Compact) << '\n';
  return 0;
}
