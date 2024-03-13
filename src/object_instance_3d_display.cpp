#include <ogre_helpers/color_material_helper.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/parse_color.h>
#include <rviz/selection/selection_handler.h>
#include <rviz/validate_floats.h>
#include <rviz/visualization_manager.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>

#include <object_instance_msgs/ObjectInstance3D.h>

#include "object_instance_3d_display.h"

namespace rviz
{

ObjectInstance3DSelectionHandler::ObjectInstance3DSelectionHandler(DisplayContext* context) : SelectionHandler(context)
{
}

ObjectInstance3DSelectionHandler::~ObjectInstance3DSelectionHandler() = default;

void ObjectInstance3DSelectionHandler::createProperties(const Picked& obj, Property* parent_property)
{
    group_property_ = new Property("ObjectInstance3D", QVariant(), "", parent_property);
    properties_.push_back(group_property_);

    id_property_ = new IntProperty("ID", -1, "", group_property_);
    id_property_->setReadOnly(true);

    class_property_ = new StringProperty("Class", "", "", group_property_);
    class_property_->setReadOnly(true);

    probability_property_ = new FloatProperty("Probability", -1.f, "", group_property_);
    probability_property_->setReadOnly(true);

    position_property_ = new VectorProperty("Position (center)", Ogre::Vector3::ZERO, "", group_property_);
    position_property_->setReadOnly(true);

    orientation_property_ = new VectorProperty("Orientation (rx, ry, rz)", Ogre::Vector3::ZERO, "", group_property_);
    orientation_property_->setReadOnly(true);
    orientation_property_->childAt(0)->setName("rx");
    orientation_property_->childAt(1)->setName("ry");
    orientation_property_->childAt(2)->setName("rz");

    size_property_ = new VectorProperty("Size", Ogre::Vector3::ZERO, "", group_property_);
    size_property_->setReadOnly(true);
    size_property_->childAt(0)->setName("Length");
    size_property_->childAt(1)->setName("Width");
    size_property_->childAt(2)->setName("Height");

    num_points_property_ = new IntProperty("Number of points", -1, "", group_property_);
    num_points_property_->setReadOnly(true);

    group_property_->expand();
}
void ObjectInstance3DSelectionHandler::setMessage(const object_instance_msgs::ObjectInstance3D& instance)
{
    instance_ = instance;
}
void ObjectInstance3DSelectionHandler::updateProperties()
{
    id_property_->setInt(instance_.id);
    class_property_->setStdString(instance_.class_name.length() > 0 ? instance_.class_name : "Not specified");
    if (instance_.class_probabilities.size() == 1)
        probability_property_->setFloat(instance_.class_probabilities[0]);
    else if (instance_.class_index < instance_.class_probabilities.size())
        probability_property_->setFloat(instance_.class_probabilities[instance_.class_index]);
    else
        probability_property_->setFloat(0);

    const auto& p = instance_.bounding_box_pose.position;
    const auto& q = instance_.bounding_box_pose.orientation;
    Ogre::Vector3 position(p.x, p.y, p.z);
    Ogre::Quaternion orientation(q.w, q.x, q.y, q.z);
    position_property_->setVector(position);
    // Yaw, pitch, roll are swapped because Ogre uses different coordinate system orientation (same as camera)
    orientation_property_->setVector(Ogre::Vector3(orientation.getYaw(false).valueDegrees(),
                                                   orientation.getPitch(false).valueDegrees(),
                                                   orientation.getRoll(false).valueDegrees()));

    size_property_->setVector(
        Ogre::Vector3(instance_.bounding_box_size.x, instance_.bounding_box_size.y, instance_.bounding_box_size.z));
    num_points_property_->setInt(
        static_cast<int>(instance_.point_cloud.data.size() / instance_.point_cloud.point_step));
}

ObjectInstance3DDisplay::ObjectInstance3DDisplay()
{

    bb_enable_property_ = new BoolProperty(
        "Bounding boxes", true, "Sets whether bounding boxes are shown or not.", this, SLOT(propertyChanged()), this);
    bb_enable_property_->setDisableChildrenIfFalse(true);

    bb_colorization_property_ = new EnumProperty("Colorization",
                                                 "By instance",
                                                 "How bounding boxes should be colorized.",
                                                 bb_enable_property_,
                                                 SLOT(propertyChanged()),
                                                 this);
    bb_colorization_property_->addOption("Uniform color", ColoringMethod::UNIFORM_COLOR);
    bb_colorization_property_->addOption("By instance", ColoringMethod::BY_INSTANCE);
    bb_colorization_property_->addOption("By class", ColoringMethod::BY_CLASS);

    bb_uniform_color_property_ = new ColorProperty("Uniform color",
                                                   Qt::white,
                                                   "Uniform color of all bounding boxes.",
                                                   bb_enable_property_,
                                                   SLOT(propertyChanged()),
                                                   this);

    bb_wire_frame_property_ = new BoolProperty("Wire frame",
                                               true,
                                               "Sets to show boxes as wire frame or solid",
                                               bb_enable_property_,
                                               SLOT(propertyChanged()),
                                               this);

    bb_enable_arrows_property_ = new BoolProperty("Enable arrows",
                                                  true,
                                                  "Whether to enable direction arrows or not",
                                                  bb_enable_property_,
                                                  SLOT(propertyChanged()),
                                                  this);

    pc_enable_property_ = new BoolProperty(
        "Point clouds", true, "Sets whether points clouds are shown or not.", this, SLOT(propertyChanged()), this);
    pc_enable_property_->setDisableChildrenIfFalse(true);

    pc_uniform_color_property_ = new ColorProperty(
        "Uniform color", Qt::white, "Uniform color of all points.", pc_enable_property_, SLOT(propertyChanged()), this);

    pc_style_property_ = new EnumProperty("Style",
                                          "Points",
                                          "Rendering mode to use, in order of computational complexity.",
                                          pc_enable_property_,
                                          SLOT(propertyChanged()),
                                          this);
    pc_style_property_->addOption("Points", PointCloud::RM_POINTS);
    pc_style_property_->addOption("Squares", PointCloud::RM_SQUARES);
    pc_style_property_->addOption("Flat Squares", PointCloud::RM_FLAT_SQUARES);
    pc_style_property_->addOption("Spheres", PointCloud::RM_SPHERES);
    pc_style_property_->addOption("Boxes", PointCloud::RM_BOXES);

    pc_world_size_property_ = new FloatProperty(
        "Size (m)", 0.01, "Point size in meters.", pc_enable_property_, SLOT(propertyChanged()), this);
    pc_world_size_property_->setMin(0.0001);

    pc_pixel_size_property_ = new FloatProperty(
        "Size (Pixels)", 3, "Point size in pixels.", pc_enable_property_, SLOT(propertyChanged()), this);
    pc_pixel_size_property_->setMin(1);

    pc_alpha_property_ = new FloatProperty("Alpha",
                                           1.0,
                                           "Amount of transparency to apply to the points.  Note that this is "
                                           "experimental and does not always look correct.",
                                           pc_enable_property_,
                                           SLOT(propertyChanged()),
                                           this);
    pc_alpha_property_->setMin(0);
    pc_alpha_property_->setMax(1);

    pc_instances_enable_property_ =
        new BoolProperty("Instances",
                         true,
                         "Sets whether points clouds (with 'is_instance' flag set to true) are shown or not.",
                         pc_enable_property_,
                         SLOT(propertyChanged()),
                         this);
    pc_instances_enable_property_->setDisableChildrenIfFalse(true);

    pc_colorization_property_ = new EnumProperty("Colorization",
                                                 "By instance",
                                                 "How instance points should be visualized.",
                                                 pc_instances_enable_property_,
                                                 SLOT(propertyChanged()),
                                                 this);
    pc_colorization_property_->addOption("By intensity", ColoringMethod::BY_INTENSITY);
    pc_colorization_property_->addOption("Uniform color", ColoringMethod::UNIFORM_COLOR);
    pc_colorization_property_->addOption("By instance", ColoringMethod::BY_INSTANCE);
    pc_colorization_property_->addOption("By class", ColoringMethod::BY_CLASS);

    pc_ni_enable_property_ =
        new BoolProperty("Non-instances",
                         true,
                         "Sets whether points clouds (with 'is_instance' flag set to false) are shown or not.",
                         pc_enable_property_,
                         SLOT(propertyChanged()),
                         this);
    pc_ni_enable_property_->setDisableChildrenIfFalse(true);

    pc_ni_colorization_property_ = new EnumProperty("Colorization",
                                                    "By instance",
                                                    "How non-instance points should be visualized.",
                                                    pc_ni_enable_property_,
                                                    SLOT(propertyChanged()),
                                                    this);
    pc_ni_colorization_property_->addOption("By intensity", ColoringMethod::BY_INTENSITY);
    pc_ni_colorization_property_->addOption("Uniform color", ColoringMethod::UNIFORM_COLOR);
    pc_ni_colorization_property_->addOption("By instance", ColoringMethod::BY_INSTANCE);
    pc_ni_colorization_property_->addOption("By class", ColoringMethod::BY_CLASS);

    pc_ni_uniform_color_property_ = new ColorProperty("Uniform color",
                                                      Qt::white,
                                                      "Uniform color of all non-instance points.",
                                                      pc_ni_enable_property_,
                                                      SLOT(propertyChanged()),
                                                      this);

    continuous_input_enable_property_ =
        new BoolProperty("Enable continuous input",
                         false,
                         "Normally when a message with object instances arrives, all visuals are cleared and all new "
                         "object instances are rendered. In this mode the old visuals are not cleared on message "
                         "arrival. They are cleared when they exceed a certain age or when a instance with the same ID "
                         "is published (see child properties).",
                         this,
                         SLOT(propertyChanged()),
                         this);

    continuous_input_delete_older_than_property_ =
        new FloatProperty("Delete instances older than",
                          0.1f,
                          "Clear all visuals which are older than X seconds.",
                          continuous_input_enable_property_,
                          SLOT(propertyChanged()),
                          this);

    continuous_input_id_based_update_property_ =
        new BoolProperty("Enable ID based update",
                         true,
                         "Clear old visuals only when a instance with the same ID arrives and render the new instance. "
                         "When a instance with the ame ID arrives but with NaN position then the visual is cleared and "
                         "no new one is rendered. Now specific instance IDs can be initiated, updated, and cleared.",
                         continuous_input_enable_property_,
                         SLOT(propertyChanged()),
                         this);

    classes_property_ =
        new Property("Classes", QVariant(), "A list of all classes seen so far.", this, SLOT(propertyChanged()), this);
}

void ObjectInstance3DDisplay::load(const Config& config)
{
    MFDClass::load(config);

    Config c = config.mapGetChild("Classes");
    for (Config::MapIterator iter = c.mapIterator(); iter.isValid(); iter.advance())
    {
        QString key = iter.currentKey();
        const Config& child = iter.currentChild();

        QColor color = parseColor(child.getValue().toString());
        createAndAddClassProperty(key.toStdString(), ColorHelper::qtToOgre(color));
    }
}

void ObjectInstance3DDisplay::onInitialize()
{
    MFDClass::onInitialize();
    propertyChanged();
}

void ObjectInstance3DDisplay::processMessage(const object_instance_msgs::ObjectInstance3DArrayConstPtr& msg)
{
    new_msgs_.push_back(msg);
}

void ObjectInstance3DDisplay::update(float wall_dt, float ros_dt)
{
    if (continuous_input_enable_property_->getBool())
    {
        for (const auto& msg : new_msgs_)
            manageInstanceContainers(msg);

        // clear instances, which are too old
        if (continuous_input_delete_older_than_property_->getFloat() > 0)
        {
            ros::Duration dt(continuous_input_delete_older_than_property_->getFloat());
            auto it = instance_containers_.begin();
            while (it != instance_containers_.end())
                if (ros::Time::now() > it->arrival_stamp + dt)
                    it = disableInstanceContainer(it);
                else
                    ++it;
        }
    }
    else if (!new_msgs_.empty())
    {
        // clear all instance containers
        auto it = instance_containers_.begin();
        while (it != instance_containers_.end())
            it = disableInstanceContainer(it);

        // add instance container for each instance
        manageInstanceContainers(new_msgs_.back());
    }

    // std::cout << "Instance Containers (used, disabled): " << instance_containers_.size() << ", "
    //           << disabled_instance_containers_.size() << std::endl;

    new_msgs_.clear();
}

std::list<InstanceContainer>::iterator
ObjectInstance3DDisplay::disableInstanceContainer(const std::list<InstanceContainer>::iterator& it)
{
    it->cloud_visual->clear();
    it->box_scene_node->setVisible(false);
    it->cloud_scene_node->setVisible(false);
    disabled_instance_containers_.push_back(*it);
    return instance_containers_.erase(it);
}

void ObjectInstance3DDisplay::updateInstanceContainer(const list<InstanceContainer>::iterator& it,
                                                      const InstanceMsgRef& msg_ref)
{
    it->msg_ref = msg_ref;
    it->arrival_stamp = ros::Time::now();
    updateBoxVisual(*it);
    updatePointCloudVisual(*it);
    applyTransforms(*it);
}

void ObjectInstance3DDisplay::manageInstanceContainers(const object_instance_msgs::ObjectInstance3DArray::ConstPtr msg)
{
    bool id_based_update = continuous_input_id_based_update_property_->getBool();

    int instance_index = 0;
    for (const auto& instance_msg : msg->instances)
    {
        InstanceMsgRef instance_msg_ref{msg, instance_index++};
        checkForNewClasses(instance_msg_ref);

        bool disable_visual = std::isnan(instance_msg.bounding_box_pose.position.x) ||
                              std::isnan(instance_msg.bounding_box_pose.position.y) ||
                              std::isnan(instance_msg.bounding_box_pose.position.z);

        auto it = std::lower_bound(
            instance_containers_.begin(), instance_containers_.end(), InstanceContainer{instance_msg_ref, scene_node_});
        if (id_based_update && it != instance_containers_.end() && it->msg_ref.getMessage().id == instance_msg.id)
        {
            if (disable_visual)
                disableInstanceContainer(it);
            else
                updateInstanceContainer(it, instance_msg_ref);
        }
        else if (!disable_visual)
        {
            if (!disabled_instance_containers_.empty())
            {
                // reuse disabled instance container
                it = instance_containers_.insert(it, disabled_instance_containers_.front());
                disabled_instance_containers_.pop_front();
            }
            else
            {
                // create and add new instance container
                InstanceContainer new_instance_container{instance_msg_ref, scene_node_};
                it = instance_containers_.insert(it, new_instance_container);
                addBoxVisualToScene(*it);
                addPointCloudVisualToScene(*it);
            }
            updateInstanceContainer(it, instance_msg_ref);
        }
    }
}

void ObjectInstance3DDisplay::addPointCloudVisualToScene(InstanceContainer& instance_container)
{
    instance_container.cloud_scene_node.reset(scene_node_->createChildSceneNode());
    instance_container.cloud_visual = std::make_shared<rviz::PointCloud>();
    instance_container.cloud_scene_node->attachObject(instance_container.cloud_visual.get());
}

void ObjectInstance3DDisplay::updatePointCloudVisual(InstanceContainer& instance_container)
{
    instance_container.cloud_scene_node->setVisible(true);

    instance_container.cloud_visual->clear();

    if (!pc_enable_)
        return;

    auto instance_msg = instance_container.msg_ref.getMessage();

    // Ignore non-instances
    if (!instance_msg.is_instance && !pc_ni_enable_)
        return;

    // Ignore instances
    if (instance_msg.is_instance && !pc_instances_enable_)
        return;

    auto mode = static_cast<PointCloud::RenderMode>(pc_style_);
    float point_size;
    if (mode == PointCloud::RM_POINTS)
    {
        point_size = pc_pixel_size_;
    }
    else
    {
        point_size = pc_world_size_;
    }

    bool per_point_alpha = findChannelIndex(instance_msg.point_cloud, "rgba") != -1;
    instance_container.cloud_visual->setRenderMode(mode);
    instance_container.cloud_visual->setAlpha(pc_alpha_, per_point_alpha);
    instance_container.cloud_visual->setDimensions(point_size, point_size, point_size);
    instance_container.cloud_visual->setAutoSize(false);

    std::vector<PointCloud::Point> cloud_points;

    size_t num_points = instance_msg.point_cloud.width * instance_msg.point_cloud.height;
    PointCloud::Point default_pt;
    default_pt.color = Ogre::ColourValue(1, 1, 1);
    default_pt.position = Ogre::Vector3::ZERO;
    cloud_points.resize(num_points, default_pt);

    int32_t xi = findChannelIndex(instance_msg.point_cloud, "x");
    int32_t yi = findChannelIndex(instance_msg.point_cloud, "y");
    int32_t zi = findChannelIndex(instance_msg.point_cloud, "z");
    int32_t ii = findChannelIndex(instance_msg.point_cloud, "intensity");

    if (xi < 0 || yi < 0 || zi < 0)
        return;

    bool intensity_available = true;
    if (ii < 0)
    {
        intensity_available = false;
        if (pc_colorization_ == ColoringMethod::BY_INTENSITY || pc_ni_colorization_ == ColoringMethod::BY_INTENSITY)
        {
            setStatus(StatusProperty::Warn, "Point cloud", "No intensity available");
        }
    }

    auto iter = cloud_points.begin();

    Ogre::ColourValue color;
    std::string material;

    size_t instance_id_color = instance_msg.id % ColorHelper::getColorMaterialNameListSize();

    if (instance_msg.is_instance)
    {
        switch (pc_colorization_)
        {
            case BY_INSTANCE:
                color = ColorHelper::getOgreColorFromList(instance_id_color);
                break;
            case UNIFORM_COLOR:
                color = pc_uniform_color_;
                break;
            case BY_CLASS:
                color = std::get<1>(object_classes_map_.find(instance_msg.class_name)->second);
                break;
            default:
                break;
        }
    }
    else
    {
        switch (pc_ni_colorization_)
        {
            case BY_INSTANCE:
                color = ColorHelper::getOgreColorFromList(instance_id_color);
                break;
            case UNIFORM_COLOR:
                color = pc_ni_uniform_color_;
                break;
            case BY_CLASS:
                color = std::get<1>(object_classes_map_.find(instance_msg.class_name)->second);
                break;
            default:
                break;
        }
    }

    const sensor_msgs::PointCloud2& cloud = instance_msg.point_cloud;

    const uint32_t xoff = cloud.fields[xi].offset;
    const uint32_t yoff = cloud.fields[yi].offset;
    const uint32_t zoff = cloud.fields[zi].offset;
    const uint32_t ioff = cloud.fields[ii].offset;
    const uint32_t point_step = cloud.point_step;
    uint8_t const* point_x = &cloud.data.front() + xoff;
    uint8_t const* point_y = &cloud.data.front() + yoff;
    uint8_t const* point_z = &cloud.data.front() + zoff;
    uint8_t const* point_i = &cloud.data.front() + ioff;

    for (int i = 0; i < cloud.width * cloud.height;
         i++, iter++, point_x += point_step, point_y += point_step, point_z += point_step, point_i += point_step)
    {
        // set position
        iter->position.x = *reinterpret_cast<const float*>(point_x);
        iter->position.y = *reinterpret_cast<const float*>(point_y);
        iter->position.z = *reinterpret_cast<const float*>(point_z);

        uint8_t intensity = 0;
        if (intensity_available)
        {
            intensity = *reinterpret_cast<const uint8_t*>(point_i);
        }

        if (!validateFloats(iter->position))
        {
            iter->position.x = 999999.0f;
            iter->position.y = 999999.0f;
            iter->position.z = 999999.0f;
        }

        bool use_intensity =
            intensity_available && ((instance_msg.is_instance && pc_colorization_ == BY_INTENSITY) ||
                                    (!instance_msg.is_instance && pc_ni_colorization_ == BY_INTENSITY));
        if (use_intensity)
        {
            // individual color for each point according to intensity
            iter->color = ColorHelper::getRainbowOgreColor(static_cast<float>(intensity) / 255.f);
        }
        else
        {
            iter->color = color;
        }
    }
    instance_container.cloud_visual->addPoints(&cloud_points.front(), cloud_points.size());
}

void ObjectInstance3DDisplay::updateBoxVisual(InstanceContainer& instance_container)
{
    // set visibility
    auto instance_msg = instance_container.msg_ref.getMessage();
    bool hide = !bb_enable_ || !instance_msg.is_instance || instance_msg.bounding_box_size.x == 0 ||
                instance_msg.bounding_box_size.y == 0 || instance_msg.bounding_box_size.z == 0;
    if (hide)
    {
        instance_container.box_scene_node->setVisible(false);
        return;
    }

    instance_container.box_scene_node->setVisible(true);
    instance_container.arrow_visual->getSceneNode()->setVisible(bb_enable_arrows_);

    Ogre::ColourValue color;
    std::string material;

    int instance_id_color = instance_msg.id % static_cast<int>(ColorHelper::getColorMaterialNameListSize());

    switch (bb_colorization_)
    {
        case BY_INSTANCE:
            color = ColorHelper::getOgreColorFromList(instance_id_color);
            material = ColorHelper::getColorMaterialNameFromList(instance_id_color, bb_wire_frame_);
            break;
        case UNIFORM_COLOR:
            color = bb_uniform_color_;
            material = bb_uniform_color_material_;
            break;
        case BY_CLASS:
        {
            auto it = object_classes_map_.find(instance_msg.class_name);
            color = std::get<1>(it->second);
            material = std::get<2>(it->second);
            break;
        }
        default:
            break;
    }

    geometry_msgs::Point p = instance_msg.bounding_box_pose.position;
    geometry_msgs::Quaternion q = instance_msg.bounding_box_pose.orientation;
    Ogre::Vector3 box_position(p.x, p.y, p.z);
    Ogre::Quaternion box_orientation(q.w, q.x, q.y, q.z);
    if (box_orientation == Ogre::Quaternion::ZERO)
        throw std::runtime_error("The quaternion is invalid, all values are zero! Set w to 1 for Identity.");
    instance_container.box_scene_node->setPosition(box_position);
    instance_container.box_scene_node->setOrientation(box_orientation);

    double x_half = instance_msg.bounding_box_size.x / 2.f;
    double y_half = instance_msg.bounding_box_size.y / 2.f;
    double z_half = instance_msg.bounding_box_size.z / 2.f;

    Ogre::AxisAlignedBox aabb(-x_half, -y_half, -z_half, x_half, y_half, z_half);
    if (bb_wire_frame_)
    {
        instance_container.solid_bounding_box_visual->setVisible(false);

        instance_container.wire_bounding_box_visual->setVisible(true);
        instance_container.wire_bounding_box_visual->setMaterial(material);
        instance_container.wire_bounding_box_visual->setupBoundingBox(aabb);
    }
    else
    {
        instance_container.wire_bounding_box_visual->setVisible(false);

        instance_container.solid_bounding_box_visual->setVisible(true);
        instance_container.solid_bounding_box_visual->setMaterial(material);
        instance_container.solid_bounding_box_visual->setupBoundingBox(aabb);
    }
    instance_container.selection_handler->setMessage(instance_msg);

    // draw direction arrow
    instance_container.arrow_visual->setPosition(Ogre::Vector3(x_half, 0, 0));
    instance_container.arrow_visual->setColor(color);
}

void ObjectInstance3DDisplay::addBoxVisualToScene(InstanceContainer& instance_container)
{
    // init scene node of box
    instance_container.box_scene_node.reset(scene_node_->createChildSceneNode());

    // add wire bounding box
    instance_container.wire_bounding_box_visual = std::make_shared<rviz::WireBoundingBox>();
    instance_container.box_scene_node->attachObject(instance_container.wire_bounding_box_visual.get());

    // add solid bounding box
    instance_container.solid_bounding_box_visual = std::make_shared<rviz::SolidBoundingBox>();
    instance_container.box_scene_node->attachObject(instance_container.solid_bounding_box_visual.get());

    // add selection handler
    instance_container.selection_handler = std::make_shared<ObjectInstance3DSelectionHandler>(context_);
    instance_container.selection_handler->addTrackedObjects(instance_container.box_scene_node.get());

    // add direction arrow
    instance_container.arrow_visual =
        std::make_shared<Arrow>(context_->getSceneManager(), instance_container.box_scene_node.get());
    instance_container.arrow_visual->set(0.3, 0.3, 0.5, 0.5);
    instance_container.arrow_visual->setOrientation(Ogre::Quaternion(Ogre::Radian(-M_PI / 2), Ogre::Vector3::UNIT_Y));
}

void ObjectInstance3DDisplay::reset()
{
    MFDClass::reset();
    for (auto& instance_container : instance_containers_)
        instance_container.removeVisuals();
    for (auto& instance_container : disabled_instance_containers_)
        instance_container.removeVisuals();
    instance_containers_.clear();
    disabled_instance_containers_.clear();
}

void ObjectInstance3DDisplay::propertyChanged()
{
    // for efficiency parse the properties just once
    bb_enable_ = bb_enable_property_->getBool();
    bb_wire_frame_ = bb_wire_frame_property_->getBool();
    bb_colorization_ = bb_colorization_property_->getOptionInt();
    bb_uniform_color_ = bb_uniform_color_property_->getOgreColor();
    bb_uniform_color_material_ = ColorHelper::createColorMaterial(bb_uniform_color_, bb_wire_frame_);
    bb_enable_arrows_ = bb_enable_arrows_property_->getBool();

    pc_enable_ = pc_enable_property_->getBool();
    pc_colorization_ = pc_colorization_property_->getOptionInt();
    pc_uniform_color_ = pc_uniform_color_property_->getOgreColor();
    pc_pixel_size_ = pc_pixel_size_property_->getFloat();
    pc_world_size_ = pc_world_size_property_->getFloat();
    pc_alpha_ = pc_alpha_property_->getFloat();
    pc_style_ = pc_style_property_->getOptionInt();
    pc_instances_enable_ = pc_instances_enable_property_->getBool();
    pc_ni_enable_ = pc_ni_enable_property_->getBool();
    pc_ni_colorization_ = pc_ni_colorization_property_->getOptionInt();
    pc_ni_uniform_color_ = pc_ni_uniform_color_property_->getOgreColor();

    for (auto& item : object_classes_map_)
    {
        Ogre::ColourValue color = std::get<3>(item.second)->getOgreColor();
        std::get<1>(item.second) = color;
        std::get<2>(item.second) = ColorHelper::createColorMaterial(color, bb_wire_frame_);
    }

    if (bb_colorization_ == ColoringMethod::UNIFORM_COLOR)
    {
        bb_uniform_color_property_->show();
    }
    else
    {
        bb_uniform_color_property_->hide();
    }

    if (static_cast<PointCloud::RenderMode>(pc_style_) == PointCloud::RM_POINTS)
    {
        pc_world_size_property_->hide();
        pc_pixel_size_property_->show();
    }
    else
    {
        pc_world_size_property_->show();
        pc_pixel_size_property_->hide();
    }

    if (pc_colorization_ == ColoringMethod::UNIFORM_COLOR)
    {
        pc_uniform_color_property_->show();
    }
    else
    {
        pc_uniform_color_property_->hide();
    }

    if (pc_ni_colorization_ == ColoringMethod::UNIFORM_COLOR)
    {
        pc_ni_uniform_color_property_->show();
    }
    else
    {
        pc_ni_uniform_color_property_->hide();
    }

    for (auto& instance_container : instance_containers_)
    {
        updateBoxVisual(instance_container);
        updatePointCloudVisual(instance_container);
        applyTransforms(instance_container);
    }
}

bool ObjectInstance3DDisplay::applyTransforms(InstanceContainer& instance_container)
{
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    const auto& instances_msg = instance_container.msg_ref.instances_msg;
    if (!context_->getFrameManager()->getTransform(instances_msg->header, position, orientation))
    {
        std::stringstream ss;
        ss << "Failed to transform from frame [" << instances_msg->header.frame_id << "] to frame ["
           << context_->getFrameManager()->getFixedFrame() << "]";
        setStatusStd(StatusProperty::Error, "Message", ss.str());
        return false;
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);

    return true;
}

void ObjectInstance3DDisplay::checkForNewClasses(const InstanceMsgRef& instance_msg_ref)
{
    const auto& instance_msg = instance_msg_ref.getMessage();
    if (object_classes_map_.find(instance_msg.class_name) == object_classes_map_.end())
    {
        // size_t color_idx = object_classes_map_.size() % ColorHelper::getColorMaterialNameListSize();
        size_t color_idx = instance_msg.class_index % ColorHelper::getColorMaterialNameListSize();
        Ogre::ColourValue color = ColorHelper::getOgreColorFromList(color_idx);
        createAndAddClassProperty(instance_msg.class_name, color);
    }
}

void ObjectInstance3DDisplay::createAndAddClassProperty(const std::string& class_name, const Ogre::ColourValue& color)
{

    auto property = new ColorProperty(QString::fromStdString(class_name),
                                      ColorHelper::ogreToQt(color),
                                      "Single class property.",
                                      classes_property_,
                                      SLOT(propertyChanged()),
                                      this);
    object_classes_map_.insert(
        {class_name, {true, color, ColorHelper::createColorMaterial(color, bb_wire_frame_), property}});
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
#include <utility>
PLUGINLIB_EXPORT_CLASS(rviz::ObjectInstance3DDisplay, rviz::Display)
