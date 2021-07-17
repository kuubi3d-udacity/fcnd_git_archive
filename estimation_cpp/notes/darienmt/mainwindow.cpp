#include "Error in " Util.relativeFilePath('/home/kuubi/ai/Udacity/git/notes/darienmt'/mainwindow.h', '/home/kuubi/ai/Udacity/git/notes/darienmt'' + '/' + Util.path('mainwindow.cpp'))": SyntaxError: Expected token `)'"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

