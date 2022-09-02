/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include "gait_parameter_gui_plugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GaitParameterGuiPlugin)

/////////////////////////////////////////////////
GaitParameterGuiPlugin::GaitParameterGuiPlugin() : GUIPlugin()
{
  // Layout
  auto mainLayout = new QVBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);

  auto frame = new QFrame();
  mainLayout->addWidget(frame);
  // mainLayout->setAlignment(Qt::AlignTop);
  this->setLayout(mainLayout);
  // Position and resize this widget
  this->move(10, 10);
  // this->resize(500, 400);
  this->setStyleSheet(
      "QFrame {background-color: rgba(100, 100, 100, 255); color: white;}");

  //Create a node for pub gait msg
  this->rosnode = new ros::NodeHandle();
  
  this->gait_param_pub = this->rosnode->advertise<unitree_legged_msgs::GaitParameter>("/gaitParam", 1000);
}

/////////////////////////////////////////////////
void GaitParameterGuiPlugin::Load(sdf::ElementPtr /*_elem*/)
{
  // Get scene
  auto scene = rendering::get_scene();
  if (!scene)
  {
    gzerr << "Scene not found" << std::endl;
    return;
  }

  // Get root visual
  auto worldVis = scene->WorldVisual();
  if (!worldVis)
  {
    gzerr << "World visual not found" << std::endl;
    return;
  }

  
  
  QLabel *pGaitTypeLabel = new QLabel(this);
  QLabel *pStanceDurationLabel = new QLabel(this);
  QLabel *pDutyFactorLabel = new QLabel(this);
  QLabel *pFullCycleLabel = new QLabel(this);
  QLabel *pContactDetectionPhaseThreshold = new QLabel(this);

  pGaitTypeLabel->setText(QStringLiteral("GaitType:"));
  pStanceDurationLabel->setText(QStringLiteral("StanceTime:"));
  pDutyFactorLabel->setText(QStringLiteral("DutyFactor:"));
  pFullCycleLabel->setText(QStringLiteral("Phase:"));
  pContactDetectionPhaseThreshold->setText(QStringLiteral("ContactPhase:"));

  auto stance_duration_fr = new QDoubleSpinBox();
  stance_duration_fr->setWrapping(true);
  stance_duration_fr->setMaximum(1.0);
  stance_duration_fr->setMinimum(0.0);
  stance_duration_fr->setSingleStep(0.05);
  
  auto stance_duration_fl = new QDoubleSpinBox();
  stance_duration_fl->setWrapping(true);
  stance_duration_fl->setMaximum(1.0);
  stance_duration_fl->setMinimum(0.0);
  stance_duration_fl->setSingleStep(0.05);
  
  auto stance_duration_hr = new QDoubleSpinBox();
  stance_duration_hr->setWrapping(true);
  stance_duration_hr->setMaximum(1.0);
  stance_duration_hr->setMinimum(0.0);
  stance_duration_hr->setSingleStep(0.05);
  
  auto stance_duration_hl = new QDoubleSpinBox();
  stance_duration_hl->setWrapping(true);
  stance_duration_hl->setMaximum(1.0);
  stance_duration_hl->setMinimum(0.0);
  stance_duration_hl->setSingleStep(0.05);
  
  auto duty_factor_fr = new QDoubleSpinBox();
  duty_factor_fr->setWrapping(true);
  duty_factor_fr->setMaximum(1.0);
  duty_factor_fr->setMinimum(0.0);
  duty_factor_fr->setSingleStep(0.05);
  
  auto duty_factor_fl = new QDoubleSpinBox();
  duty_factor_fl->setWrapping(true);
  duty_factor_fl->setMaximum(1.0);
  duty_factor_fl->setMinimum(0.0);
  duty_factor_fl->setSingleStep(0.05);
  
  auto duty_factor_hr = new QDoubleSpinBox();
  duty_factor_hr->setWrapping(true);
  duty_factor_hr->setMaximum(1.0);
  duty_factor_hr->setMinimum(0.0);
  duty_factor_hr->setSingleStep(0.05);
  
  auto duty_factor_hl = new QDoubleSpinBox();
  duty_factor_hl->setWrapping(true);
  duty_factor_hl->setMaximum(1.0);
  duty_factor_hl->setMinimum(0.0);
  duty_factor_hl->setSingleStep(0.05);
  
  auto init_phase_full_cycle_fr = new QDoubleSpinBox();
  init_phase_full_cycle_fr->setWrapping(true);
  init_phase_full_cycle_fr->setMaximum(1.0);
  init_phase_full_cycle_fr->setMinimum(0.0);
  init_phase_full_cycle_fr->setSingleStep(0.05);
  
  auto init_phase_full_cycle_fl = new QDoubleSpinBox();
  init_phase_full_cycle_fl->setWrapping(true);
  init_phase_full_cycle_fl->setMaximum(1.0);
  init_phase_full_cycle_fl->setMinimum(0.0);
  init_phase_full_cycle_fl->setSingleStep(0.05);
  
  auto init_phase_full_cycle_hr = new QDoubleSpinBox();
  init_phase_full_cycle_hr->setWrapping(true);
  init_phase_full_cycle_hr->setMaximum(1.0);
  init_phase_full_cycle_hr->setMinimum(0.0);
  init_phase_full_cycle_hr->setSingleStep(0.05);
  
  auto init_phase_full_cycle_hl = new QDoubleSpinBox();
  init_phase_full_cycle_hl->setWrapping(true);
  init_phase_full_cycle_hl->setMaximum(1.0);
  init_phase_full_cycle_hl->setMinimum(0.0);
  init_phase_full_cycle_hl->setSingleStep(0.05);
  
  auto contact_detection_phase_threshold = new QDoubleSpinBox();
  contact_detection_phase_threshold->setWrapping(true);
  contact_detection_phase_threshold->setMaximum(1.0);
  contact_detection_phase_threshold->setMinimum(0.0);
  contact_detection_phase_threshold->setSingleStep(0.05);
  


  auto gaitType = new QComboBox();
  //定义步态类型列表
  QStringList gaitList;
  gaitList << "trot" << "advanced_trot" << "stand" << "threestand";

  QList<Gait> gaitData;
  gaitData.append({
    QString("trot"),
    QList<double>{0.3,0.3,0.3,0.3},
    QList<double>{0.6,0.6,0.6,0.6},
    QList<double>{0.5,0.,0.,0.5},
    QList<int>{1,1,1,1},
    0.5});
  gaitData.append({
    QString("advanced_trot"),
    QList<double>{0.6, 0.6, 0.6, 0.6},
    QList<double>{0.6,0.6,0.6,0.6},
    QList<double>{0.5,0.,0.,0.5},
    QList<int>{1,1,1,1},
    0.5});
  gaitData.append({
    QString("stand"),
    QList<double>{0.3,0.3,0.3,0.3},
    QList<double>{1.,1.,1.,1.},
    QList<double>{0.,0.,0.,0.},
    QList<int>{1,1,1,1},
    0.1});
  gaitData.append({
    QString("threestand"),
    QList<double>{0.3,0.3,0.3,0.3},
    QList<double>{1.,1.,0.,1.},
    QList<double>{0.,0.,0.,0.},
    QList<int>{1,1,1,1},
    0.1});
  QMap<QString,QVariant> map_gait_data;
  // QMap<QString,QVariant> copy_gait_data = map_gait_data;

  for (int i = 0; i < gaitData.size(); i++)
  {
    map_gait_data.insert(gaitList[i],QVariant::fromValue(gaitData[i]));
  }
  // std::cout<< map_gait_data[0].value<gait>().contact_detection_phase_threshold<<endl;
  //将步态类型列表绑定 QComboBox 控件，添加项
  
  auto cb = [=](const QString &)
  {
    
    stance_duration_fr->setValue(gaitType->currentData().value<Gait>().stance_duration[0]);
    stance_duration_fl->setValue(gaitType->currentData().value<Gait>().stance_duration[1]);
    stance_duration_hr->setValue(gaitType->currentData().value<Gait>().stance_duration[2]);
    stance_duration_hl->setValue(gaitType->currentData().value<Gait>().stance_duration[3]);

    duty_factor_fr->setValue(gaitType->currentData().value<Gait>().duty_factor[0]);
    duty_factor_fl->setValue(gaitType->currentData().value<Gait>().duty_factor[1]);
    duty_factor_hr->setValue(gaitType->currentData().value<Gait>().duty_factor[2]);
    duty_factor_hl->setValue(gaitType->currentData().value<Gait>().duty_factor[3]);

    
    init_phase_full_cycle_fr->setValue(gaitType->currentData().value<Gait>().init_full_cycle[0]);
    init_phase_full_cycle_fl->setValue(gaitType->currentData().value<Gait>().init_full_cycle[1]);
    init_phase_full_cycle_hr->setValue(gaitType->currentData().value<Gait>().init_full_cycle[2]);
    init_phase_full_cycle_hl->setValue(gaitType->currentData().value<Gait>().init_full_cycle[3]);

    contact_detection_phase_threshold->setValue(gaitType->currentData().value<Gait>().contact_detection_phase_threshold);
    //std::cout<< gaitType->currentData().value<gait>().contact_detection_phase_threshold<<endl;
  };

  this->connect(gaitType, static_cast<void (QComboBox::*)(const QString &)>(
    &QComboBox::currentIndexChanged), cb);
  foreach(const QString &str,map_gait_data.keys())
    gaitType->addItem(str,map_gait_data.value(str));
  gaitType->setCurrentIndex(1);

  this->connect(stance_duration_fr, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    stance_duration_fr->setValue(_v);
  });

  this->connect(stance_duration_fl, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    stance_duration_fl->setValue(_v);
  });

  this->connect(stance_duration_hr, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    stance_duration_hr->setValue(_v);
  });

  this->connect(stance_duration_hl, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    stance_duration_hl->setValue(_v);
  });

  this->connect(duty_factor_fr, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    duty_factor_fr->setValue(_v);
  });

  this->connect(duty_factor_fl, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    duty_factor_fl->setValue(_v);
  });

  this->connect(duty_factor_hr, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    duty_factor_hr->setValue(_v);
  });

  this->connect(duty_factor_hl, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    duty_factor_hl->setValue(_v);
  });

  this->connect(init_phase_full_cycle_fr, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    init_phase_full_cycle_fr->setValue(_v);
  });

  this->connect(init_phase_full_cycle_fl, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    init_phase_full_cycle_fl->setValue(_v);
  });

  this->connect(init_phase_full_cycle_hr, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    init_phase_full_cycle_hr->setValue(_v);
  });

  this->connect(init_phase_full_cycle_hl, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    init_phase_full_cycle_hl->setValue(_v);
  });

  this->connect(contact_detection_phase_threshold, static_cast<void (QDoubleSpinBox::*)(double)>(
      &QDoubleSpinBox::valueChanged), [=](double _v)
  {
    contact_detection_phase_threshold->setValue(_v);
  });
  // Create a push button, and connect it to the OnButton function
  auto saveButton = new QPushButton(tr("Save it"));

  auto resetButton = new QPushButton(tr("Reset it"));

  connect(resetButton, &QPushButton::clicked,gaitType,[=]() {
    stance_duration_fr->setValue(gaitType->currentData().value<Gait>().stance_duration[0]);
    stance_duration_fl->setValue(gaitType->currentData().value<Gait>().stance_duration[1]);
    stance_duration_hr->setValue(gaitType->currentData().value<Gait>().stance_duration[2]);
    stance_duration_hl->setValue(gaitType->currentData().value<Gait>().stance_duration[3]);

    duty_factor_fr->setValue(gaitType->currentData().value<Gait>().duty_factor[0]);
    duty_factor_fl->setValue(gaitType->currentData().value<Gait>().duty_factor[1]);
    duty_factor_hr->setValue(gaitType->currentData().value<Gait>().duty_factor[2]);
    duty_factor_hl->setValue(gaitType->currentData().value<Gait>().duty_factor[3]);

    
    init_phase_full_cycle_fr->setValue(gaitType->currentData().value<Gait>().init_full_cycle[0]);
    init_phase_full_cycle_fl->setValue(gaitType->currentData().value<Gait>().init_full_cycle[1]);
    init_phase_full_cycle_hr->setValue(gaitType->currentData().value<Gait>().init_full_cycle[2]);
    init_phase_full_cycle_hl->setValue(gaitType->currentData().value<Gait>().init_full_cycle[3]);

    contact_detection_phase_threshold->setValue(gaitType->currentData().value<Gait>().contact_detection_phase_threshold);
  });
  
  connect(saveButton, &QPushButton::clicked,gaitType,[=]() {
    unitree_legged_msgs::GaitParameter gaitMsg;
    const Gait gaitParam = gaitType->currentData().value<Gait>();

    gaitMsg.gaitName = gaitParam.gait_name.toStdString();

    gaitMsg.stanceDuration[0] = static_cast<float>(stance_duration_fr->value());
    gaitMsg.stanceDuration[1] = static_cast<float>(stance_duration_fl->value());
    gaitMsg.stanceDuration[2] = static_cast<float>(stance_duration_hr->value());
    gaitMsg.stanceDuration[3] = static_cast<float>(stance_duration_hl->value());

    gaitMsg.dutyFactor[0] = static_cast<float>(duty_factor_fr->value());
    gaitMsg.dutyFactor[1] = static_cast<float>(duty_factor_fl->value());
    gaitMsg.dutyFactor[2] = static_cast<float>(duty_factor_hr->value());
    gaitMsg.dutyFactor[3] = static_cast<float>(duty_factor_hl->value());

    gaitMsg.initFullCycle[0] = static_cast<float>(init_phase_full_cycle_fr->value());
    gaitMsg.initFullCycle[1] = static_cast<float>(init_phase_full_cycle_fl->value());
    gaitMsg.initFullCycle[2] = static_cast<float>(init_phase_full_cycle_hr->value());
    gaitMsg.initFullCycle[3] = static_cast<float>(init_phase_full_cycle_hl->value());

    gaitMsg.initLegState[0] = gaitParam.init_leg_state[0];
    gaitMsg.initLegState[1] = gaitParam.init_leg_state[1];
    gaitMsg.initLegState[2] = gaitParam.init_leg_state[2];
    gaitMsg.initLegState[3] = gaitParam.init_leg_state[3];

    gaitMsg.contactDetectionPhaseThreshold = static_cast<float>(contact_detection_phase_threshold->value());

    ROS_INFO("gait_param_contact:");

    this->gait_param_pub.publish(gaitMsg);
  });


  QGridLayout *mainLayout = new QGridLayout();
  mainLayout->addWidget(pGaitTypeLabel,0,0);
  mainLayout->addWidget(gaitType,0,1);
  mainLayout->addWidget(pStanceDurationLabel, 1, 0);
  QHBoxLayout *stLayout = new QHBoxLayout();
  stLayout->addWidget(new QLabel("<b>FR</b>"));
  stLayout->addWidget(stance_duration_fr);
  mainLayout->addLayout(stLayout,1,1);
  mainLayout->addWidget(new QLabel("<b>FL</b>"),1,2);
  mainLayout->addWidget(stance_duration_fl, 1, 3);
  mainLayout->addWidget(new QLabel("<b>HR</b>"),1,4);
  mainLayout->addWidget(stance_duration_hr, 1, 5);
  mainLayout->addWidget(new QLabel("<b>HL</b>"),1,6);
  mainLayout->addWidget(stance_duration_hl, 1, 7);
  mainLayout->addWidget(pDutyFactorLabel, 2, 0);
  QHBoxLayout *duLayout = new QHBoxLayout();
  duLayout->addWidget(new QLabel("<b>FR</b>"));
  duLayout->addWidget(duty_factor_fr);
  mainLayout->addLayout(duLayout,2,1);
  mainLayout->addWidget(new QLabel("<b>FL</b>"),2,2);
  mainLayout->addWidget(duty_factor_fl, 2, 3);
  mainLayout->addWidget(new QLabel("<b>HR</b>"),2,4);
  mainLayout->addWidget(duty_factor_hr, 2, 5);
  mainLayout->addWidget(new QLabel("<b>HL</b>"),2,6);
  mainLayout->addWidget(duty_factor_hl, 2, 7);
  mainLayout->addWidget(pFullCycleLabel, 3, 0);
  QHBoxLayout *phLayout = new QHBoxLayout();
  phLayout->addWidget(new QLabel("<b>FR</b>"));
  phLayout->addWidget(init_phase_full_cycle_fr);
  mainLayout->addLayout(phLayout,3,1);
  mainLayout->addWidget(new QLabel("<b>FL</b>"),3,2);
  mainLayout->addWidget(init_phase_full_cycle_fl, 3, 3);
  mainLayout->addWidget(new QLabel("<b>HR</b>"),3,4);
  mainLayout->addWidget(init_phase_full_cycle_hr, 3, 5);
  mainLayout->addWidget(new QLabel("<b>HL</b>"),3,6);
  mainLayout->addWidget(init_phase_full_cycle_hl, 3, 7);
  mainLayout->addWidget(pContactDetectionPhaseThreshold, 4, 0);
  mainLayout->addWidget(contact_detection_phase_threshold, 4, 1);
  mainLayout->addWidget(resetButton,5,6);
  mainLayout->addWidget(saveButton,5,7);
  mainLayout->setSpacing(15);
  mainLayout->setMargin(15);
  mainLayout->setAlignment(Qt::AlignTop);

  
  auto frame = this->findChild<QFrame *>();
  frame->setLayout(mainLayout);
}


/////////////////////////////////////////////////
GaitParameterGuiPlugin::~GaitParameterGuiPlugin()
{
}

