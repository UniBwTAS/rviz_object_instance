#pragma once

#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <ogre_primitives/solid_bounding_box.h>
#include <ogre_primitives/wire_frame_bounding_box.h>
#include <rviz/selection/selection_handler.h>

#include <object_instance_msgs/ObjectInstance3DArray.h>

#include <utility>

namespace rviz
{
class IntProperty;
class BoolProperty;
class EnumProperty;
class FloatProperty;
class ColorProperty;
class StringProperty;
class VectorProperty;
class Picked;
class Arrow;

class ObjectInstance3DDisplay;

inline int32_t findChannelIndex(const sensor_msgs::PointCloud2& cloud, const std::string& channel)
{
    for (size_t i = 0; i < cloud.fields.size(); ++i)
    {
        if (cloud.fields[i].name == channel)
        {
            return i;
        }
    }

    return -1;
}

class ObjectInstance3DSelectionHandler : public SelectionHandler
{
  public:
    explicit ObjectInstance3DSelectionHandler(DisplayContext* context);
    ~ObjectInstance3DSelectionHandler() override;

    void createProperties(const Picked& obj, Property* parent_property) override;
    void setMessage(const object_instance_msgs::ObjectInstance3D& instance);
    void updateProperties() override;

  private:
    object_instance_msgs::ObjectInstance3D instance_;

    Property* group_property_ = nullptr;
    IntProperty* id_property_ = nullptr;
    StringProperty* class_property_ = nullptr;
    FloatProperty* probability_property_ = nullptr;
    VectorProperty* position_property_ = nullptr;
    VectorProperty* orientation_property_ = nullptr;
    VectorProperty* size_property_ = nullptr;
    IntProperty* num_points_property_ = nullptr;
};

struct InstanceMsgRef
{
  public:
    InstanceMsgRef(const object_instance_msgs::ObjectInstance3DArray::ConstPtr& instances_msg, int instance_index)
        : instances_msg(instances_msg), instance_index(instance_index)
    {
    }

    bool operator<(const InstanceMsgRef& other) const
    {
        return getMessage().id < other.getMessage().id;
    }

    const object_instance_msgs::ObjectInstance3D& getMessage() const
    {
        return instances_msg->instances[instance_index];
    }

    object_instance_msgs::ObjectInstance3DArray::ConstPtr instances_msg;
    int instance_index{-1};
};

class InstanceContainer
{
  public:
    InstanceContainer(InstanceMsgRef msg_ref, Ogre::SceneNode* parent_scene_node)
        : msg_ref(std::move(msg_ref)), parent_scene_node(parent_scene_node){};

    Ogre::SceneNode* parent_scene_node;
    std::shared_ptr<Ogre::SceneNode> box_scene_node;
    std::shared_ptr<Ogre::SceneNode> cloud_scene_node;

    // store pointer to instance messages visualized by this container: list of (object list + list position)
    InstanceMsgRef msg_ref;
    ros::Time arrival_stamp;

    std::shared_ptr<SolidBoundingBox> solid_bounding_box_visual;
    std::shared_ptr<WireBoundingBox> wire_bounding_box_visual;
    std::shared_ptr<ObjectInstance3DSelectionHandler> selection_handler;
    std::shared_ptr<Arrow> arrow_visual;
    std::shared_ptr<PointCloud> cloud_visual;

    void removeVisuals()
    {
        arrow_visual.reset(); // removes itself from scene graph on destruction
        box_scene_node->removeAndDestroyAllChildren();
        cloud_scene_node->removeAndDestroyAllChildren();
        solid_bounding_box_visual.reset();
        wire_bounding_box_visual.reset();
        cloud_visual.reset();
        selection_handler.reset();
        parent_scene_node->removeChild(box_scene_node.get());
        parent_scene_node->removeChild(cloud_scene_node.get());
    }

    bool operator<(const InstanceContainer& other) const
    {
        return msg_ref < other.msg_ref;
    }
};

class ObjectInstance3DDisplay : public MessageFilterDisplay<object_instance_msgs::ObjectInstance3DArray>
{
  public:
    enum ColoringMethod
    {
        BY_INTENSITY,
        UNIFORM_COLOR,
        BY_INSTANCE,
        BY_CLASS,
    };

    Q_OBJECT
  public:
    ObjectInstance3DDisplay();

    void reset() override;

    void update(float wall_dt, float ros_dt) override;

  protected:
    void load(const Config& config) override;

    /** @brief Do initialization. Overridden from MessageFilterDisplay. */
    void onInitialize() override;

    /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
    void processMessage(const object_instance_msgs::ObjectInstance3DArrayConstPtr& msg) override;

    void manageInstanceContainers(const object_instance_msgs::ObjectInstance3DArray::ConstPtr msg);
    std::list<InstanceContainer>::iterator disableInstanceContainer(const std::list<InstanceContainer>::iterator& it);
    void updateInstanceContainer(const std::list<InstanceContainer>::iterator& it, const InstanceMsgRef& msg_ref);
    void addPointCloudVisualToScene(InstanceContainer& instance_container);
    void updatePointCloudVisual(InstanceContainer& instance_container);
    void addBoxVisualToScene(InstanceContainer& instance_container);
    void updateBoxVisual(InstanceContainer& instance_container);
    bool applyTransforms(InstanceContainer& instance_container);
    void checkForNewClasses(const InstanceMsgRef& instance_msg_ref);

  private Q_SLOTS:
    void propertyChanged();

  private:
    void createAndAddClassProperty(const std::string& class_name, const Ogre::ColourValue& color);

  private:
    std::list<InstanceContainer> instance_containers_;
    std::list<InstanceContainer> disabled_instance_containers_;
    std::vector<object_instance_msgs::ObjectInstance3DArrayConstPtr> new_msgs_;

    // bounding box properties & values
    BoolProperty* bb_enable_property_;
    BoolProperty* bb_wire_frame_property_;
    EnumProperty* bb_colorization_property_;
    ColorProperty* bb_uniform_color_property_;
    BoolProperty* bb_enable_arrows_property_;
    bool bb_enable_;
    bool bb_wire_frame_;
    int bb_colorization_;
    Ogre::ColourValue bb_uniform_color_;
    std::string bb_uniform_color_material_;
    bool bb_enable_arrows_;

    // point cloud properties & values
    BoolProperty* pc_enable_property_;
    EnumProperty* pc_colorization_property_;
    ColorProperty* pc_uniform_color_property_;
    FloatProperty* pc_pixel_size_property_;
    FloatProperty* pc_world_size_property_;
    FloatProperty* pc_alpha_property_;
    EnumProperty* pc_style_property_;
    BoolProperty* pc_instances_enable_property_;
    BoolProperty* pc_ni_enable_property_;
    EnumProperty* pc_ni_colorization_property_;
    ColorProperty* pc_ni_uniform_color_property_;
    bool pc_enable_;
    int pc_colorization_;
    Ogre::ColourValue pc_uniform_color_;
    float pc_pixel_size_;
    float pc_world_size_;
    float pc_alpha_;
    int pc_style_;
    bool pc_instances_enable_;
    bool pc_ni_enable_;
    int pc_ni_colorization_;
    Ogre::ColourValue pc_ni_uniform_color_;

    BoolProperty* continuous_input_enable_property_;
    FloatProperty* continuous_input_delete_older_than_property_;
    BoolProperty* continuous_input_id_based_update_property_;

    // class related properties
    Property* classes_property_;
    std::map<std::string, std::tuple<bool, Ogre::ColourValue, std::string, ColorProperty*>> object_classes_map_;
};

} // namespace rviz