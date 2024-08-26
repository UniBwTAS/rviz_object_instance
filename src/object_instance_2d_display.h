#pragma once

#include <QObject>

#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#include <rviz/image/image_display_base.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/render_panel.h>

#include <message_filters/time_synchronizer.h>

#include <object_instance_msgs/ObjectInstance2DArray.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>

namespace Ogre
{
class SceneNode;
class Rectangle2D;
} // namespace Ogre

namespace rviz
{

class IntProperty;
class BoolProperty;
class EnumProperty;
class FloatProperty;
class ColorProperty;
class StringProperty;
class VectorProperty;
class PointCloud;
class Picked;
class Arrow;

/**
 * \class ObjectInstance2DDisplay
 *
 */
class ObjectInstance2DDisplay : public ImageDisplayBase
{
  public:
    enum ColoringMethod
    {
        UNIFORM_COLOR,
        BY_INSTANCE,
        BY_CLASS,
    };

    enum SegmentationType
    {
        INSTANCE_SEGMENTATION,
        SEMANTIC_SEGMENTATION,
        PANOPTIC_SEGMENTATION
    };

    Q_OBJECT
  public:
    ObjectInstance2DDisplay();
    ~ObjectInstance2DDisplay() override;

  public Q_SLOTS:
    virtual void updateNormalizeOptions();
    void propertyChanged();

  protected:
    // Overrides from Display
    void onInitialize() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;
    void onEnable() override;
    void onDisable() override;
    void subscribe() override;
    void unsubscribe() override;

    /* This is called by incomingMessage(). */
    void processMessage(const sensor_msgs::ImageConstPtr& msg) override;
    void processCamInfoMessage(const sensor_msgs::CameraInfo::ConstPtr& msg);

    void processInstancesMessage(const object_instance_msgs::ObjectInstance2DArrayConstPtr& msg);

    void processSyncedMessages(const sensor_msgs::ImageConstPtr& image_msg,
                               const object_instance_msgs::ObjectInstance2DArrayConstPtr& instance_msg);

    void
    enqueueMsg(const std::pair<sensor_msgs::ImageConstPtr, object_instance_msgs::ObjectInstance2DArrayConstPtr>& p);
    void checkForNewClasses();
    std::tuple<bool, QColor, ColorProperty*> addClassToList(uint16_t class_index, const std::string& class_name);

    ros::Subscriber caminfo_sub_;
    sensor_msgs::ImageConstPtr debayer(const sensor_msgs::Image::ConstPtr& raw_msg);
    sensor_msgs::ImageConstPtr rectify(const sensor_msgs::ImageConstPtr& image_msg);

    Ogre::SceneManager* img_scene_manager_{};

    ROSImageTexture texture_;

    RenderPanel* render_panel_{};

    std::shared_ptr<message_filters::Subscriber<object_instance_msgs::ObjectInstance2DArray>> instances_sub_;
    std::deque<std::pair<sensor_msgs::ImageConstPtr, object_instance_msgs::ObjectInstance2DArrayConstPtr>> msg_queue_;
    std::pair<sensor_msgs::ImageConstPtr, object_instance_msgs::ObjectInstance2DArrayConstPtr> last_msgs_;
    uint32_t inst_messages_received_;

  private:
    void clear();

    Ogre::SceneNode* img_scene_node_{};
    Ogre::Rectangle2D* screen_rect_{};
    Ogre::MaterialPtr material_;

    RosTopicProperty* instances_topic_property_;

    Property* img_property_;
    BoolProperty* img_normalize_property_;
    FloatProperty* img_min_property_;
    FloatProperty* img_max_property_;
    bool got_float_image_;

    EnumProperty* debayer_property_;
    EnumProperty* rectify_property_;
    image_geometry::PinholeCameraModel model_;
    sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
    boost::mutex caminfo_mutex_;

    IntProperty* img_median_buffer_size_property_;
    BoolProperty* bb_enable_property_;
    EnumProperty* bb_colorization_property_;
    ColorProperty* bb_uniform_color_property_;
    BoolProperty* bb_show_labels_property_;
    bool bb_enable_{};
    int bb_colorization_{};
    QColor bb_uniform_color_;
    bool bb_show_labels_{};

    BoolProperty* seg_enable_property_;
    EnumProperty* seg_type_property_;
    FloatProperty* seg_alpha_property_;
    EnumProperty* seg_colorization_property_;
    ColorProperty* seg_uniform_color_property_;
    bool seg_enable_{};
    int seg_type_{};
    float seg_alpha_{};
    int seg_colorization_{};
    QColor seg_uniform_color_;

    FloatProperty* hide_if_score_below_property_;
    float hide_if_score_below_{};

    // class related properties
    Property* classes_property_;
    std::map<uint16_t, std::tuple<bool, QColor, ColorProperty*>> object_classes_map_;
};

} // namespace rviz