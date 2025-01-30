#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreTextureManager.h>

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/render_panel.h>

#include <boost/dynamic_bitset.hpp>

#include <object_instance_msgs/ObjectInstance2DArray.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_common.h>

#include <ogre_helpers/color_material_helper.h>

#include "object_instance_2d_display.h"

namespace rviz
{

enum
{
    Debayer_Bilinear = 0,
    Debayer_EdgeAware = 1,
    Debayer_EdgeAwareWeighted = 2,
    Debayer_VNG = 3,
};

enum
{
    Interpolation_NN = 0,
    Interpolation_Linear = 1,
    Interpolation_Cubic = 2,
    Interpolation_Lanczos4 = 3,
    Interpolation_Off = 4,
};

ObjectInstance2DDisplay::ObjectInstance2DDisplay() : ImageDisplayBase(), texture_(), inst_messages_received_(0)
{
    // remove properties added by ImageDisplayBase in order to push them all into a single parent property for a clearer
    // tree structure
    removeChildren();
    topic_property_ = new RosTopicProperty("Image Topic",
                                           "",
                                           QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                           "sensor_msgs::Image topic to subscribe to.",
                                           this,
                                           SLOT(updateTopic()));

    instances_topic_property_ = new RosTopicProperty(
        "Instances Topic",
        "",
        QString::fromStdString(ros::message_traits::datatype<object_instance_msgs::ObjectInstance2DArray>()),
        "object_instance_msgs::ObjectInstance2DArray topic to subscribe to.",
        this,
        SLOT(updateTopic()),
        this);

    img_property_ = new Property("Image", QVariant(), "Image related settings.", this);

    transport_property_ = new EnumProperty(
        "Transport Hint", "raw", "Preferred method of sending images.", img_property_, SLOT(updateTopic()), this);

    connect(
        transport_property_, SIGNAL(requestOptions(EnumProperty*)), this, SLOT(fillTransportOptionList(EnumProperty*)));

    queue_size_property_ =
        new IntProperty("Queue Size",
                        2,
                        "Advanced: set the size of the incoming message queue.  Increasing this "
                        "is useful if your incoming TF data is delayed significantly from your"
                        " image data, but it can greatly increase memory usage if the messages are big.",
                        img_property_,
                        SLOT(updateQueueSize()),
                        this);
    queue_size_property_->setMin(1);

    transport_property_->setStdString("raw");

    unreliable_property_ =
        new BoolProperty("Unreliable", false, "Prefer UDP topic transport", img_property_, SLOT(updateTopic()), this);

    // own properties

    debayer_property_ =
        new EnumProperty("Debayering Algorithm", "Bilinear", "Debayering algorithm.", this, SLOT(propertyChanged()), this);
    debayer_property_->addOption("Bilinear", Debayer_Bilinear);
    debayer_property_->addOption("EdgeAware", Debayer_EdgeAware);
    debayer_property_->addOption("EdgeAwareWeighted", Debayer_EdgeAwareWeighted);
    debayer_property_->addOption("VNG", Debayer_VNG);

    rectify_property_ = new EnumProperty("Rectify Algorithm",
                                         "Linear",
                                         "Interpolation algorithm between source image pixels.",
                                         this,
                                         SLOT(propertyChanged()),
                                         this);
    rectify_property_->addOption("Nearest neighbor", Interpolation_NN);
    rectify_property_->addOption("Linear", Interpolation_Linear);
    rectify_property_->addOption("Cubic", Interpolation_Cubic);
    rectify_property_->addOption("Lanczos4", Interpolation_Lanczos4);
    rectify_property_->addOption("Off", Interpolation_Off);

    img_normalize_property_ =
        new BoolProperty("Normalize Range",
                         true,
                         "If set to true, will try to estimate the range of possible values from the received images.",
                         img_property_,
                         SLOT(updateNormalizeOptions()),
                         this);

    img_min_property_ = new FloatProperty("Min Value",
                                          0.0,
                                          "Value which will be displayed as black.",
                                          img_property_,
                                          SLOT(updateNormalizeOptions()),
                                          this);

    img_max_property_ = new FloatProperty("Max Value",
                                          1.0,
                                          "Value which will be displayed as white.",
                                          img_property_,
                                          SLOT(updateNormalizeOptions()),
                                          this);

    img_median_buffer_size_property_ = new IntProperty("Median window",
                                                       5,
                                                       "Window size for median filter used for computing min/max.",
                                                       img_property_,
                                                       SLOT(updateNormalizeOptions()),
                                                       this);

    bb_enable_property_ = new BoolProperty(
        "Bounding boxes", true, "Bounding boxes related settings.", this, SLOT(propertyChanged()), this);

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
    bb_show_labels_property_ = new BoolProperty(
        "Show labels", true, "Whether to show labels or not.", bb_enable_property_, SLOT(propertyChanged()), this);

    seg_enable_property_ = new BoolProperty(
        "Masks", true, "Instance or Semantic mask related settings.", this, SLOT(propertyChanged()), this);

    seg_type_property_ = new EnumProperty("Type",
                                          "Instance Segmentation",
                                          "Whether to visualize instance Segmentation or semantic segmentation",
                                          seg_enable_property_,
                                          SLOT(propertyChanged()),
                                          this);
    seg_type_property_->addOption("Instance Segmentation", SegmentationType::INSTANCE_SEGMENTATION);
    seg_type_property_->addOption("Semantic Segmentation", SegmentationType::SEMANTIC_SEGMENTATION);
    seg_type_property_->addOption("Panoptic Segmentation", SegmentationType::PANOPTIC_SEGMENTATION);

    seg_alpha_property_ =
        new FloatProperty("Alpha", 0.5, "Alpha of mask overlay.", seg_enable_property_, SLOT(propertyChanged()), this);

    seg_colorization_property_ = new EnumProperty("Colorization",
                                                  "By instance",
                                                  "How instance masks should be colorized.",
                                                  seg_enable_property_,
                                                  SLOT(propertyChanged()),
                                                  this);
    seg_colorization_property_->addOption("Uniform color", ColoringMethod::UNIFORM_COLOR);
    seg_colorization_property_->addOption("By instance", ColoringMethod::BY_INSTANCE);
    seg_colorization_property_->addOption("By class", ColoringMethod::BY_CLASS);

    seg_uniform_color_property_ = new ColorProperty(
        "Uniform color", Qt::red, "Uniform color of all masks.", seg_enable_property_, SLOT(propertyChanged()), this);

    hide_if_score_below_property_ = new FloatProperty("Hide if Score Below",
                                                      0.f,
                                                      "Hide box and mask if score is below this value.",
                                                      this,
                                                      SLOT(propertyChanged()),
                                                      this);
    hide_if_score_below_property_->setMin(0.f);
    hide_if_score_below_property_->setMax(1.f);

    classes_property_ =
        new Property("Classes", QVariant(), "A list of all classes seen so far.", this, SLOT(propertyChanged()), this);

    got_float_image_ = false;
}

void ObjectInstance2DDisplay::onInitialize()
{
    ImageDisplayBase::onInitialize();
    {
        static uint32_t count = 0;
        std::stringstream ss;
        ss << "ObjectInstance2DDisplay" << count++;
        img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
    }

    img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

    {
        static int count = 0;
        std::stringstream ss;
        ss << "ObjectInstance2DDisplayObject" << count++;

        screen_rect_ = new Ogre::Rectangle2D(true);
        screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
        screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

        ss << "Material";
        material_ = Ogre::MaterialManager::getSingleton().create(
            ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material_->setSceneBlending(Ogre::SBT_REPLACE);
        material_->setDepthWriteEnabled(false);
        material_->setReceiveShadows(false);
        material_->setDepthCheckEnabled(false);

        material_->getTechnique(0)->setLightingEnabled(false);
        Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_.getTexture()->getName());
        tu->setTextureFiltering(Ogre::TFO_NONE);

        material_->setCullingMode(Ogre::CULL_NONE);
        Ogre::AxisAlignedBox aabInf;
        aabInf.setInfinite();
        screen_rect_->setBoundingBox(aabInf);
        screen_rect_->setMaterial(material_->getName());
        img_scene_node_->attachObject(screen_rect_);
    }

    render_panel_ = new RenderPanel();
    render_panel_->getRenderWindow()->setAutoUpdated(false);
    render_panel_->getRenderWindow()->setActive(false);

    render_panel_->resize(640, 480);
    render_panel_->initialize(img_scene_manager_, context_);

    setAssociatedWidget(render_panel_);

    render_panel_->setAutoRender(false);
    render_panel_->setOverlaysEnabled(false);
    render_panel_->getCamera()->setNearClipDistance(0.01f);

    updateNormalizeOptions();
    propertyChanged();
}

ObjectInstance2DDisplay::~ObjectInstance2DDisplay()
{
    if (initialized())
    {
        delete render_panel_;
        delete screen_rect_;
        img_scene_node_->getParentSceneNode()->removeAndDestroyChild(img_scene_node_->getName());
    }
}

void ObjectInstance2DDisplay::onEnable()
{
    subscribe();
    render_panel_->getRenderWindow()->setActive(true);
}

void ObjectInstance2DDisplay::onDisable()
{
    render_panel_->getRenderWindow()->setActive(false);
    unsubscribe();
    reset();
}

void ObjectInstance2DDisplay::subscribe()
{
    std::string target_frame = fixed_frame_.toStdString();
    ImageDisplayBase::enableTFFilter(target_frame);
    ImageDisplayBase::subscribe();

    if (!isEnabled() || topic_property_->getTopicStd().empty())
    {
        return;
    }

    if (!instances_topic_property_->getTopic().isEmpty())
    {
        try
        {
            instances_sub_.reset(new message_filters::Subscriber<object_instance_msgs::ObjectInstance2DArray>(
                update_nh_, instances_topic_property_->getTopicStd(), 10));

            instances_sub_->registerCallback(boost::bind(&ObjectInstance2DDisplay::processInstancesMessage, this, _1));

            setStatus(StatusProperty::Ok, "Instances Topic", "OK");
        }
        catch (ros::Exception& e)
        {
            setStatus(StatusProperty::Error, "Instances Topic", QString("Error subscribing: ") + e.what());
        }
    }

    try
    {
        const std::string caminfo_topic = image_transport::getCameraInfoTopic(topic_property_->getTopicStd());
        caminfo_sub_ = update_nh_.subscribe(caminfo_topic, 1, &ObjectInstance2DDisplay::processCamInfoMessage, this);
    }
    catch (ros::Exception& e)
    {
        setStatus(StatusProperty::Error, "Camera Info", QString("Error subscribing: ") + e.what());
    }
}

void ObjectInstance2DDisplay::unsubscribe()
{
    ImageDisplayBase::unsubscribe();
    instances_sub_.reset();
    caminfo_sub_.shutdown();

    boost::mutex::scoped_lock lock(caminfo_mutex_);
    current_caminfo_.reset();
}

void ObjectInstance2DDisplay::updateNormalizeOptions()
{
    if (got_float_image_)
    {
        bool normalize = img_normalize_property_->getBool();

        img_normalize_property_->setHidden(false);
        img_min_property_->setHidden(normalize);
        img_max_property_->setHidden(normalize);
        img_median_buffer_size_property_->setHidden(!normalize);

        texture_.setNormalizeFloatImage(normalize, img_min_property_->getFloat(), img_max_property_->getFloat());
        texture_.setMedianFrames(img_median_buffer_size_property_->getInt());
    }
    else
    {
        img_normalize_property_->setHidden(true);
        img_min_property_->setHidden(true);
        img_max_property_->setHidden(true);
        img_median_buffer_size_property_->setHidden(true);
    }
}

void ObjectInstance2DDisplay::clear()
{
    texture_.clear();

    if (render_panel_->getCamera())
        render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void ObjectInstance2DDisplay::update(float wall_dt, float ros_dt)
{
    Q_UNUSED(wall_dt)
    Q_UNUSED(ros_dt)
    try
    {
        texture_.update();

        // make sure the aspect ratio of the image is preserved
        auto win_width = static_cast<float>(render_panel_->width());
        auto win_height = static_cast<float>(render_panel_->height());

        auto img_width = static_cast<float>(texture_.getWidth());
        auto img_height = static_cast<float>(texture_.getHeight());

        if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0)
        {
            float img_aspect = img_width / img_height;
            float win_aspect = win_width / win_height;

            if (img_aspect > win_aspect)
            {
                screen_rect_->setCorners(
                    -1.0f, 1.0f * win_aspect / img_aspect, 1.0f, -1.0f * win_aspect / img_aspect, false);
            }
            else
            {
                screen_rect_->setCorners(
                    -1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect, -1.0f, false);
            }
        }

        render_panel_->getRenderWindow()->update();
    }
    catch (UnsupportedImageEncoding& e)
    {
        setStatus(StatusProperty::Error, "Image", e.what());
    }
}

void ObjectInstance2DDisplay::reset()
{
    clear();
    ImageDisplayBase::reset();

    inst_messages_received_ = 0;
    setStatus(StatusProperty::Warn, "Instances", "No InstanceArray received");

    setStatus(
        StatusLevel::Warn,
        "Sync",
        "Unable to synchronize image and instances. Are both messages published with exactly the same timestamp?");

    // We explicitly do not reset current_caminfo_ here: If we are subscribed to a latched caminfo topic,
    // we will not receive another message after reset, i.e. the caminfo could not be recovered.
    // Thus, we reset caminfo only if unsubscribing.

    const std::string topic = topic_property_->getTopicStd();
    if (!topic.empty())
    {
        const std::string caminfo_topic = image_transport::getCameraInfoTopic(topic);
        boost::mutex::scoped_lock lock(caminfo_mutex_);
        if (!current_caminfo_)
            setStatus(StatusProperty::Warn,
                      "Camera Info",
                      "No CameraInfo received on [" + QString::fromStdString(caminfo_topic) +
                          "].\nTopic may not exist.");
    }
}

/* This is called by incomingMessage(). */
void ObjectInstance2DDisplay::processMessage(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::ImageConstPtr msg_debayer = debayer(msg);
    sensor_msgs::ImageConstPtr msg_rectify = rectify(msg_debayer);
    enqueueMsg({msg_rectify, object_instance_msgs::ObjectInstance2DArrayConstPtr()});
}

void ObjectInstance2DDisplay::processCamInfoMessage(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(caminfo_mutex_);
    current_caminfo_ = msg;
    setStatus(StatusProperty::Ok, "Camera Info", "received");
}

void ObjectInstance2DDisplay::processInstancesMessage(const object_instance_msgs::ObjectInstance2DArrayConstPtr& msg)
{
    ++inst_messages_received_;
    setStatus(StatusProperty::Ok, "Instances", QString::number(inst_messages_received_) + " InstanceArrays received");
    enqueueMsg({sensor_msgs::ImageConstPtr(), msg});
}

void ObjectInstance2DDisplay::processSyncedMessages(
    const sensor_msgs::ImageConstPtr& image_msg,
    const object_instance_msgs::ObjectInstance2DArrayConstPtr& instance_msg)
{

    setStatus(StatusLevel::Ok, "Sync", "OK");

    last_msgs_ = {image_msg, instance_msg};
    checkForNewClasses();

    bool got_float_image = image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                           image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                           image_msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
                           image_msg->encoding == sensor_msgs::image_encodings::MONO16;

    if (got_float_image != got_float_image_)
    {
        got_float_image_ = got_float_image;
        updateNormalizeOptions();
    }

    // get background image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // prepare semantic/instance masks
    bool instance_mask_available = instance_msg->instance_mask.width > 0 && instance_msg->instance_mask.height > 0;
    bool semantic_mask_available = instance_msg->semantic_mask.width > 0 && instance_msg->semantic_mask.height > 0;
    bool draw_semantic_overlay =
        seg_enable_ && (seg_type_ == SegmentationType::SEMANTIC_SEGMENTATION || seg_type_ == PANOPTIC_SEGMENTATION) &&
        semantic_mask_available;
    bool draw_instance_overlay =
        seg_enable_ && (seg_type_ == SegmentationType::INSTANCE_SEGMENTATION || seg_type_ == PANOPTIC_SEGMENTATION) &&
        instance_mask_available;
    cv_bridge::CvImagePtr cv_semantic_ptr, cv_instance_ptr;
    if (draw_instance_overlay)
    {
        try
        {
            cv_instance_ptr = cv_bridge::toCvCopy(instance_msg->instance_mask, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Instance mask must be MONO16: %s", e.what());
            return;
        }
    }
    bool colorize_instance_mask_by_class = draw_instance_overlay && seg_colorization_ == ColoringMethod::BY_CLASS;
    if (draw_semantic_overlay || (colorize_instance_mask_by_class && semantic_mask_available))
    {
        try
        {
            cv_semantic_ptr = cv_bridge::toCvCopy(instance_msg->semantic_mask, sensor_msgs::image_encodings::MONO16);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Semantic mask must be MONO16: %s", e.what());
            return;
        }
    }

    // create a mapping from instance ids to (class index, class probability)
    std::map<uint16_t, std::pair<uint16_t, float>> instance_index_to_class_index_map;
    // create a mapping class index to instance index
    for (const auto& instance : instance_msg->instances)
    {
        float probability = instance.class_probabilities.size() == 1 ?
                                instance.class_probabilities[0] :
                                instance.class_probabilities[instance.class_index];
        instance_index_to_class_index_map.insert({instance.id, {instance.class_index, probability}});
    }

    // draw semantic/instance mask overlay
    int cached_class_index = -1;
    QColor cached_q_color;
    cv::Mat segmentation_overlay = cv_ptr->image.clone();
    // segmentation_overlay = 0;
    if (draw_semantic_overlay || draw_instance_overlay)
    {
        // iterate over all pixels
        for (int row = 0; row < cv_ptr->image.rows; row++)
        {
            for (int col = 0; col < cv_ptr->image.cols; col++)
            {
                boost::optional<QColor> q_color;

                if (draw_semantic_overlay)
                {
                    uint16_t class_index = cv_semantic_ptr->image.at<uint16_t>(row, col);
                    if (cached_class_index >= 0 && class_index == cached_class_index)
                        q_color = cached_q_color;
                    else
                    {
                        q_color = std::get<1>(object_classes_map_.find(class_index)->second);
                        cached_class_index = class_index;
                        cached_q_color = *q_color;
                    }
                }
                if (draw_instance_overlay)
                {
                    uint16_t instance_index = cv_instance_ptr->image.at<uint16_t>(row, col);
                    if (instance_index > 0)
                    {
                        // check if we want to hide this instance segment because the score is too low
                        bool hide_because_score_is_too_low = false;
                        uint16_t class_index = 0;
                        if (hide_if_score_below_ > 0.0f)
                        {
                            auto it = instance_index_to_class_index_map.find(instance_index);
                            if (it == instance_index_to_class_index_map.end())
                                throw std::runtime_error("Unable to find instance index in map: " +
                                                         std::to_string(instance_index));
                            class_index = it->second.first; // store for later
                            float probability = it->second.second;
                            hide_because_score_is_too_low = probability < hide_if_score_below_;
                        }

                        if (!hide_because_score_is_too_low)
                        {
                            switch (seg_colorization_)
                            {
                                case BY_INSTANCE:
                                {
                                    int color_idx = instance_index % static_cast<int>(ColorHelper::getColorListSize());
                                    q_color = ColorHelper::getColorFromList(color_idx);
                                    break;
                                }
                                case BY_CLASS:
                                {
                                    if (hide_if_score_below_ == 0)
                                    {
                                        // class index was not obtained yet
                                        if (semantic_mask_available)
                                            class_index =
                                                cv_semantic_ptr->image.at<uint16_t>(row, col); // more efficient
                                        else
                                        {
                                            auto it = instance_index_to_class_index_map.find(instance_index);
                                            if (it == instance_index_to_class_index_map.end())
                                                throw std::runtime_error("Unable to find instance index in map: " +
                                                                         std::to_string(instance_index));
                                            class_index = it->second.first;
                                        }
                                    }
                                    auto it = object_classes_map_.find(class_index);
                                    if (it == object_classes_map_.end())
                                        std::cerr << "Mask: Class was not saved to database so far: " << class_index
                                                  << std::endl;
                                    q_color = std::get<1>(it->second);
                                    break;
                                }
                                case UNIFORM_COLOR:
                                {
                                    q_color = seg_uniform_color_;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (q_color)
                {
                    auto& pixel = segmentation_overlay.at<cv::Vec3b>(row, col);
                    pixel.val[0] = q_color->blue();
                    pixel.val[1] = q_color->green();
                    pixel.val[2] = q_color->red();
                }
            }
        }
        cv::addWeighted(cv_ptr->image, (1 - seg_alpha_), segmentation_overlay, seg_alpha_, 0, cv_ptr->image);
    }

    for (const auto& instance : instance_msg->instances)
    {
        // bounding boxes
        if (bb_enable_)
        {
            float probability = instance.class_probabilities.size() == 1 ?
                                    instance.class_probabilities[0] :
                                    instance.class_probabilities[instance.class_index];
            if (probability < hide_if_score_below_)
                continue;

            // get desired color
            QColor q_color;
            switch (bb_colorization_)
            {
                case BY_INSTANCE:
                {
                    int color_idx = instance.id % static_cast<int>(ColorHelper::getColorListSize());
                    q_color = ColorHelper::getColorFromList(color_idx);
                    break;
                }
                case BY_CLASS:
                {
                    auto it = object_classes_map_.find(instance.class_index);
                    if (it == object_classes_map_.end())
                        std::cerr << "Bbox: Class was not saved to database so far: " << instance.class_index
                                  << std::endl;
                    q_color = std::get<1>(it->second);
                    break;
                }
                case UNIFORM_COLOR:
                {
                    q_color = bb_uniform_color_;
                    break;
                }
            }
            cv::Scalar color = CV_RGB(q_color.red(), q_color.green(), q_color.blue());

            // draw
            int x1 = static_cast<int>(instance.bounding_box_min_x);
            int y1 = static_cast<int>(instance.bounding_box_min_y);
            int x2 = static_cast<int>(instance.bounding_box_max_x);
            int y2 = static_cast<int>(instance.bounding_box_max_y);
            std::string label = instance.class_name;
            cv::rectangle(cv_ptr->image, {x1, y1}, {x2, y2}, color, 4);
            if (bb_show_labels_)
            {
                bool use_black_text = (q_color.red() * 0.299 + q_color.green() * 0.587 + q_color.blue() * 0.114) > 150;
                int baseline = 0;
                cv::Size t_size = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 2, 2, &baseline);
                cv::rectangle(cv_ptr->image, {x1, y1}, {x1 + t_size.width + 3, y1 + t_size.height + 4}, color, -1);
                cv::putText(cv_ptr->image,
                            label,
                            {x1, y1 + t_size.height + 4},
                            cv::FONT_HERSHEY_PLAIN,
                            2,
                            use_black_text ? CV_RGB(0, 0, 0) : CV_RGB(255, 255, 255),
                            2);
            }
        }
    }

    texture_.addMessage(cv_ptr->toImageMsg());
}

void ObjectInstance2DDisplay::enqueueMsg(
    const std::pair<sensor_msgs::ImageConstPtr, object_instance_msgs::ObjectInstance2DArrayConstPtr>& p)
{
    bool msg_found = false;
    for (auto it = msg_queue_.begin(); it != msg_queue_.end(); it++)
    {
        if (p.second && it->first && it->first->header.stamp == p.second->header.stamp)
        {
            // received detections
            processSyncedMessages(it->first, p.second);
            msg_queue_.erase(it);
            msg_found = true;
            break;
        }
        else if (p.first && it->second && it->second->header.stamp == p.first->header.stamp)
        {
            // received image
            processSyncedMessages(p.first, it->second);
            msg_queue_.erase(it);
            msg_found = true;
            break;
        }
    }
    if (!msg_found)
    {
        msg_queue_.push_back(p);
        if (msg_queue_.size() > 100)
        {
            msg_queue_.pop_front();
        }
    }
}

void ObjectInstance2DDisplay::propertyChanged()
{
    // bounding boxes
    bb_enable_ = bb_enable_property_->getBool();
    bb_colorization_ = bb_colorization_property_->getOptionInt();
    bb_uniform_color_ = bb_uniform_color_property_->getColor();
    bb_show_labels_ = bb_show_labels_property_->getBool();

    for (auto& item : object_classes_map_)
    {
        QColor color = std::get<2>(item.second)->getColor();
        std::get<1>(item.second) = color;
    }

    if (bb_colorization_ == ColoringMethod::UNIFORM_COLOR)
        bb_uniform_color_property_->show();
    else
        bb_uniform_color_property_->hide();

    // masks
    seg_enable_ = seg_enable_property_->getBool();
    seg_type_ = seg_type_property_->getOptionInt();
    seg_colorization_ = seg_colorization_property_->getOptionInt();
    seg_uniform_color_ = seg_uniform_color_property_->getColor();
    seg_alpha_ = seg_alpha_property_->getFloat();
    if (seg_type_ == SegmentationType::INSTANCE_SEGMENTATION || seg_type_ == SegmentationType::PANOPTIC_SEGMENTATION)
    {
        seg_colorization_property_->show();
        if (seg_colorization_ == ColoringMethod::UNIFORM_COLOR)
            seg_uniform_color_property_->show();
        else
            seg_uniform_color_property_->hide();
    }
    else
    {
        seg_colorization_property_->hide();
        seg_uniform_color_property_->hide();
    }

    // hide
    hide_if_score_below_ = hide_if_score_below_property_->getFloat();

    if (last_msgs_.first && last_msgs_.second)
    {
        processSyncedMessages(last_msgs_.first, last_msgs_.second);
    }
}

void ObjectInstance2DDisplay::checkForNewClasses()
{
    // search for new classes in semantic mask classes
    const auto& semantic_class_indices = last_msgs_.second->semantic_class_indices;
    const auto& semantic_class_names = last_msgs_.second->semantic_class_names;
    if (semantic_class_indices.size() != semantic_class_names.size())
        throw std::runtime_error("Sizes of class indices and class names must match");
    for (int i = 0; i < semantic_class_indices.size(); i++)
        addClassToList(semantic_class_indices[i], semantic_class_names[i]);

    // search for new classes in bounding box instances
    for (const auto& instance : last_msgs_.second->instances)
        addClassToList(instance.class_index, instance.class_name);
}

std::tuple<bool, QColor, ColorProperty*> ObjectInstance2DDisplay::addClassToList(uint16_t class_index,
                                                                                 const std::string& class_name)
{
    auto it = object_classes_map_.find(class_index);
    if (it == object_classes_map_.end())
    {
        int color_idx = class_index % static_cast<int>(ColorHelper::getColorListSize());
        QColor color = ColorHelper::getColorFromList(color_idx);
        auto property = new ColorProperty(QString::fromStdString(class_name),
                                          color,
                                          "Single class property.",
                                          classes_property_,
                                          SLOT(propertyChanged()),
                                          this);

        return object_classes_map_.insert({class_index, {true, color, property}}).first->second;
    }
    else
        return it->second;
}

sensor_msgs::ImageConstPtr ObjectInstance2DDisplay::debayer(const sensor_msgs::Image::ConstPtr& raw_msg)
{
    namespace enc = sensor_msgs::image_encodings;

    int bit_depth = enc::bitDepth(raw_msg->encoding);
    //@todo Fix as soon as bitDepth fixes it
    if (raw_msg->encoding == enc::YUV422)
        bit_depth = 8;

    if (enc::isMono(raw_msg->encoding) || enc::isColor(raw_msg->encoding))
    {
        setStatus(StatusProperty::Ok, "Debayer", "OK");
        return raw_msg;
    }
    else if (enc::isBayer(raw_msg->encoding))
    {
        int type = bit_depth == 8 ? CV_8U : CV_16U;
        const cv::Mat bayer(raw_msg->height,
                            raw_msg->width,
                            CV_MAKETYPE(type, 1),
                            const_cast<uint8_t*>(&raw_msg->data[0]),
                            raw_msg->step);

        sensor_msgs::ImagePtr color_msg = boost::make_shared<sensor_msgs::Image>();
        color_msg->header = raw_msg->header;
        color_msg->height = raw_msg->height;
        color_msg->width = raw_msg->width;
        color_msg->encoding = bit_depth == 8 ? enc::BGR8 : enc::BGR16;
        color_msg->step = color_msg->width * 3 * (bit_depth / 8);
        color_msg->data.resize(color_msg->height * color_msg->step);

        cv::Mat color(color_msg->height, color_msg->width, CV_MAKETYPE(type, 3), &color_msg->data[0], color_msg->step);

        int algorithm = debayer_property_->getOptionInt();
        if (algorithm == Debayer_EdgeAware || algorithm == Debayer_EdgeAwareWeighted)
        {
            // These algorithms are not in OpenCV yet
            if (raw_msg->encoding != enc::BAYER_GRBG8)
            {
                setStatus(StatusProperty::Warn,
                          "Debayer",
                          "Edge aware algorithms currently only support GRBG8 Bayer. "
                          "Falling back to bilinear interpolation.");
                algorithm = Debayer_Bilinear;
            }
            else
            {
                // if (algorithm == Debayer_EdgeAware) TODO: fix
                //     image_proc::debayerEdgeAware(bayer, color);
                // else
                //     image_proc::debayerEdgeAwareWeighted(bayer, color);
            }
        }
        if (algorithm == Debayer_Bilinear || algorithm == Debayer_VNG)
        {
            int code = -1;
            if (raw_msg->encoding == enc::BAYER_RGGB8 || raw_msg->encoding == enc::BAYER_RGGB16)
                code = cv::COLOR_BayerBG2BGR;
            else if (raw_msg->encoding == enc::BAYER_BGGR8 || raw_msg->encoding == enc::BAYER_BGGR16)
                code = cv::COLOR_BayerRG2BGR;
            else if (raw_msg->encoding == enc::BAYER_GBRG8 || raw_msg->encoding == enc::BAYER_GBRG16)
                code = cv::COLOR_BayerGR2BGR;
            else if (raw_msg->encoding == enc::BAYER_GRBG8 || raw_msg->encoding == enc::BAYER_GRBG16)
                code = cv::COLOR_BayerGB2BGR;

            if (algorithm == Debayer_VNG)
                code += cv::COLOR_BayerBG2BGR_VNG - cv::COLOR_BayerBG2BGR;

            try
            {
                cv::cvtColor(bayer, color, code);
            }
            catch (cv::Exception& e)
            {
                setStatus(StatusProperty::Warn,
                          "Debayer",
                          QString("cvtColor error: '%1', bayer code: %2, width %3, height %4")
                              .arg(e.what())
                              .arg(code)
                              .arg(bayer.cols)
                              .arg(bayer.rows));
                return raw_msg;
            }
        }

        setStatus(StatusProperty::Ok, "Debayer", "OK");
        return color_msg;
    }
    else if (raw_msg->encoding == enc::YUV422)
    {
        // Use cv_bridge to convert to BGR8
        sensor_msgs::ImagePtr color_msg;
        try
        {
            color_msg = cv_bridge::toCvCopy(raw_msg, enc::BGR8)->toImageMsg();
        }
        catch (cv_bridge::Exception& e)
        {
            setStatus(StatusProperty::Warn, "Debayer", QString("cv_bridge conversion error: '%1'").arg(e.what()));
            return raw_msg;
        }

        setStatus(StatusProperty::Ok, "Debayer", "OK");
        return color_msg;
    }
    else if (raw_msg->encoding == enc::TYPE_8UC3)
    {
        // 8UC3 does not specify a color encoding. Is it BGR, RGB, HSV, XYZ, LUV...?
        setStatus(StatusProperty::Warn,
                  "Debayer",
                  QString("Raw image topic '%s' has ambiguous encoding '8UC3'. The "
                          "source should set the encoding to 'bgr8' or 'rgb8'.") +
                      topic_property_->getTopic());
    }
    else
    {
        setStatus(StatusProperty::Warn,
                  "Debayer",
                  QString("Raw image topic '%1' has unsupported encoding '%2'")
                      .arg(QString::fromStdString(topic_property_->getTopicStd()))
                      .arg(QString::fromStdString(raw_msg->encoding)));
    }

    return raw_msg;
}

sensor_msgs::ImageConstPtr ObjectInstance2DDisplay::rectify(const sensor_msgs::ImageConstPtr& image_msg)
{
    sensor_msgs::CameraInfo::ConstPtr info_msg;
    {
        boost::mutex::scoped_lock lock(caminfo_mutex_);
        info_msg = current_caminfo_;
    }

    if (!info_msg || !image_msg || rectify_property_->getOptionInt() == Interpolation_Off)
        return image_msg;

    // Verify camera is actually calibrated
    if (info_msg->K[0] == 0.0)
    {
        setStatus(StatusProperty::Warn,
                  "Rectify",
                  QString("Camera publishing '%s' is uncalibrated").arg(topic_property_->getTopicStd().c_str()));
        return image_msg;
    }

    // If zero distortion, just pass the message along
    bool zero_distortion = true;
    for (size_t i = 0; i < info_msg->D.size(); ++i)
    {
        if (info_msg->D[i] != 0.0)
        {
            zero_distortion = false;
            break;
        }
    }
    // This will be true if D is empty/zero sized
    if (zero_distortion)
    {
        setStatus(StatusProperty::Ok, "Rectify", "OK");
        return image_msg;
    }

    // Update the camera model
    model_.fromCameraInfo(info_msg);

    // Create cv::Mat views onto both buffers
    const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat rect;

    // Rectify and publish
    int interpolation = rectify_property_->getOptionInt();
    model_.rectifyImage(image, rect, interpolation);

    // Allocate new rectified image message
    setStatus(StatusProperty::Ok, "Rectify", "OK");
    return cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::ObjectInstance2DDisplay, rviz::Display)