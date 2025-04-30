  #include <QVBoxLayout>
  #include <rviz_common/display_context.hpp>
  #include <moveit_object_creator/object_panel.hpp>

  namespace moveit_object_creator
  {
  ObjectPanel::ObjectPanel(QWidget * parent) : Panel(parent)
  {
    // Create a label and a button, displayed vertically (the V in VBox means vertical)
    const auto layout = new QVBoxLayout(this);

    // Label
    label_ = new QLabel("MoveIt Object Creator", this);
    label_->setAlignment(Qt::AlignCenter);
    layout->addWidget(label_);

    // Option selector
    comboBox_ = new QComboBox(this);
    comboBox_->addItem("Shelf");
    comboBox_->addItem("Round table");
    comboBox_->addItem("Square table");
    layout->addWidget(comboBox_);

    // Text input fields
    nameInput_ = new QLineEdit(this);
    nameInput_->setPlaceholderText("Object name");
    layout->addWidget(nameInput_);

    textInput1_ = new QLineEdit(this);
    textInput1_->setPlaceholderText("Shelf width");
    layout->addWidget(textInput1_);
    
    textInput2_ = new QLineEdit(this);
    textInput2_->setPlaceholderText("Shelf lenght");
    layout->addWidget(textInput2_);

    textInput3_ = new QLineEdit(this);
    textInput3_->setPlaceholderText("Shelf height");
    layout->addWidget(textInput3_);
    
    // Number selector
    numberSelector_ = new QSpinBox(this);
    numberSelector_->setRange(0, 100);
    numberSelector_->setValue(10);
    layout->addWidget(numberSelector_);

    textInput4_ = new QLineEdit(this);
    textInput4_->setPlaceholderText("Shelf thickness");
    layout->addWidget(textInput4_);

    // Shelf height adjustment
    shelf_height_ = new QLineEdit(this);
    shelf_height_->setPlaceholderText("Shelf height offset");
    layout->addWidget(shelf_height_);
    
    // Button
    button_ = new QPushButton("Publish!", this);
    layout->addWidget(button_);

    // Connect the event of when the button is released to our callback,
    // so pressing the button results in the callback being called.
    QObject::connect(button_, &QPushButton::released, this, &ObjectPanel::buttonActivated);
    QObject::connect(comboBox_, QOverload<int>::of(&QComboBox::currentIndexChanged),
                  this, &ObjectPanel::onComboBoxChanged);
  }

  ObjectPanel::~ObjectPanel() = default;

  void ObjectPanel::onInitialize()
  {
    // Access the abstract ROS Node and
    // in the process lock it for exclusive use until the method is done.
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
    // (as per normal rclcpp code)
    rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
    planning_scene_diff_publisher_ = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose", 10, std::bind(&ObjectPanel::topicCallback, this, std::placeholders::_1));
  }

  void ObjectPanel::topicCallback(const geometry_msgs::msg::PoseStamped& pose)
  {
    // We can use the data in the message to update the label
    object_pose_ = pose;
    reference_frame_ = pose.header.frame_id;
  }

  geometry_msgs::msg::Pose ObjectPanel::applyPoseOffset(const geometry_msgs::msg::Pose& base_pose, double dx, double dy, double dz) {
    tf2::Transform base_tf;
    tf2::fromMsg(base_pose, base_tf);

    tf2::Vector3 local_offset(dx, dy, dz);
    tf2::Vector3 world_offset = tf2::quatRotate(base_tf.getRotation(), local_offset);

    geometry_msgs::msg::Pose result = base_pose;
    result.position.x += world_offset.x();
    result.position.y += world_offset.y();
    result.position.z += world_offset.z();

    return result;
  }

  geometry_msgs::msg::Pose ObjectPanel::rotatePoseAroundZ(const geometry_msgs::msg::Pose& pose, double angle_rad) {
    tf2::Quaternion q_rot;
    q_rot.setRPY(0, 0, angle_rad);

    tf2::Quaternion q_orig;
    tf2::fromMsg(pose.orientation, q_orig);

    tf2::Quaternion q_new = q_orig * q_rot;
    q_new.normalize();

    geometry_msgs::msg::Pose result = pose;
    result.orientation = tf2::toMsg(q_new);
    return result;
  }

  // When the widget's button is pressed, this callback is triggered,
  // and then we publish a new message on our topic.
  void ObjectPanel::buttonActivated()
  {
    QString selected = comboBox_->currentText();

    if (selected == "Shelf") {
      moveit_msgs::msg::AttachedCollisionObject attached_object;
      attached_object.object.header.frame_id = reference_frame_;
      attached_object.object.id = nameInput_->text().toStdString();

      double width = textInput1_->text().toDouble();     // X direction
      double depth = textInput2_->text().toDouble();     // Y direction
      double height = textInput3_->text().toDouble();    // Z direction
      int number_of_shelves = numberSelector_->value();
      double thickness = textInput4_->text().toDouble(); // Wall thickness

      // Back wall — width x thickness x height
      shape_msgs::msg::SolidPrimitive back_wall;
      back_wall.type = back_wall.BOX;
      back_wall.dimensions = {thickness, width, height};
      attached_object.object.primitives.push_back(back_wall);
      attached_object.object.primitive_poses.push_back(
          applyPoseOffset(object_pose_.pose, -depth / 2.0 + thickness / 2.0, 0, (height / 2.0)));

      // Left wall — thickness x depth x height
      shape_msgs::msg::SolidPrimitive left_wall;
      left_wall.type = left_wall.BOX;
      left_wall.dimensions = {depth, thickness, height};
      attached_object.object.primitives.push_back(left_wall);
      attached_object.object.primitive_poses.push_back(
          applyPoseOffset(object_pose_.pose, 0, -width / 2.0 - thickness / 2.0, (height / 2.0)));

      // Right wall — thickness x depth x height
      shape_msgs::msg::SolidPrimitive right_wall;
      right_wall.type = right_wall.BOX;
      right_wall.dimensions = {depth, thickness, height};
      attached_object.object.primitives.push_back(right_wall);
      attached_object.object.primitive_poses.push_back(
          applyPoseOffset(object_pose_.pose, 0, width / 2.0 + thickness / 2.0, (height / 2.0)));

      // Horizontal shelf planes — width x depth x thickness
      for (int i = 0; i < number_of_shelves + 2; ++i) {
        shape_msgs::msg::SolidPrimitive shelf;
        shelf.type = shelf.BOX;
        shelf.dimensions = {depth, width + (thickness*2), thickness};
        attached_object.object.primitives.push_back(shelf);
      
        double z_offset;
        if (i == number_of_shelves + 1) {
          // Top shelf — place it so its **top** aligns with the outer edge of side walls
          z_offset = height - (thickness / 2.0);
        } else {
          // Regular shelf — center placed normally
          z_offset = (thickness / 2.0) + i * (height / (number_of_shelves + 1));
        }
      
        attached_object.object.primitive_poses.push_back(
            applyPoseOffset(object_pose_.pose, 0, 0, z_offset));
      }

      double shelf_height_offset = shelf_height_->text().toDouble();
      for (int i = 0; i < number_of_shelves; i++) {
        int shelf_index = i + 4; // Offset by 3 for the walls
        attached_object.object.primitive_poses[shelf_index].position.z += shelf_height_offset;
      }  

      attached_object.object.operation = attached_object.object.ADD;

      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(attached_object.object);
      planning_scene.is_diff = true;
      planning_scene_diff_publisher_->publish(planning_scene);

    } else if (selected == "Round table") {
      moveit_msgs::msg::AttachedCollisionObject attached_object;
      /* The header must contain a valid TF frame*/
      attached_object.object.header.frame_id = reference_frame_;
      /* The id of the object */
      attached_object.object.id = nameInput_->text().toStdString();

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[0] = textInput1_->text().toDouble();
      primitive.dimensions[1] = textInput2_->text().toDouble();

      attached_object.object.primitives.push_back(primitive);
      attached_object.object.primitive_poses.push_back(object_pose_.pose);

      attached_object.object.operation = attached_object.object.ADD;

      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(attached_object.object);
      planning_scene.is_diff = true;
      planning_scene_diff_publisher_->publish(planning_scene);

    } else if (selected == "Square table") {
      moveit_msgs::msg::AttachedCollisionObject attached_object;
      /* The header must contain a valid TF frame*/
      attached_object.object.header.frame_id = reference_frame_;
      /* The id of the object */
      attached_object.object.id = nameInput_->text().toStdString();

      /* Define a box to be attached */
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = textInput1_->text().toDouble();
      primitive.dimensions[1] = textInput2_->text().toDouble();
      primitive.dimensions[2] = textInput3_->text().toDouble();

      attached_object.object.primitives.push_back(primitive);
      attached_object.object.primitive_poses.push_back(object_pose_.pose);

      attached_object.object.operation = attached_object.object.ADD;

      moveit_msgs::msg::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(attached_object.object);
      planning_scene.is_diff = true;
      planning_scene_diff_publisher_->publish(planning_scene);
    }
  }

  void ObjectPanel::onComboBoxChanged(int index)
  {
    QString selected = comboBox_->itemText(index);

    if (selected == "Shelf") {
      textInput1_->setPlaceholderText("Shelf width");
      textInput1_->show();
      textInput2_->setPlaceholderText("Shelf length");
      textInput2_->show();
      textInput3_->setPlaceholderText("Shelf height");
      textInput3_->show();
      numberSelector_->show();
      textInput4_->setPlaceholderText("Shelf thickness");
      textInput4_->show();
    } else if (selected == "Round table") {
      textInput1_->setPlaceholderText("Table height");
      textInput1_->show();
      textInput2_->setPlaceholderText("Table radius");
      textInput2_->show();
      textInput3_->hide();
      textInput4_->hide();
      numberSelector_->hide();
    } else if (selected == "Square table") {
      textInput1_->setPlaceholderText("Table width");
      textInput1_->show();
      textInput2_->setPlaceholderText("Table length");
      textInput2_->show();
      textInput3_->setPlaceholderText("Table height");
      textInput3_->show();
      textInput4_->hide();
      numberSelector_->hide();
    }
  }

  }  // namespace moveit_object_creator

  #include <pluginlib/class_list_macros.hpp>
  PLUGINLIB_EXPORT_CLASS(moveit_object_creator::ObjectPanel, rviz_common::Panel)