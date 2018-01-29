#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{
  class RosGstStream
  {
    public:
      RosGstStream()
      {
        GstPad *audiopad;

        _sub = _nh.subscribe("audio", 10, &RosGstStream::onAudio, this);

        _loop = g_main_loop_new(NULL, false);

        _pipeline = gst_pipeline_new("app_pipeline");
        _source = gst_element_factory_make("appsrc", "app_source");
        gst_bin_add( GST_BIN(_pipeline), _source);

        g_signal_connect(_source, "need-data", G_CALLBACK(cb_need_data),this);

        _parser = gst_element_factory_make("opusparse", "parser");
        g_signal_connect(_parser, "pad-added", G_CALLBACK(cb_newpad),this);
        gst_bin_add( GST_BIN(_pipeline), _parser);
        gst_element_link(_source, _parser);

        _audio = gst_bin_new("audiobin");

        _pay = gst_element_factory_make("rtpopuspay", "rtppay");
        audiopad = gst_element_get_static_pad(_pay, "sink");

        _sink = gst_element_factory_make("udpsink", "streamsink");
        g_object_set(G_OBJECT(_sink), "host", "0.0.0.0", NULL);
        g_object_set(G_OBJECT(_sink), "port", 8004, NULL);

        gst_bin_add_many( GST_BIN(_audio), _pay, _sink, NULL);
        gst_element_link_many(_pay, _sink, NULL);
        gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
        gst_object_unref(audiopad);

        gst_bin_add(GST_BIN(_pipeline), _audio);

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );

        _paused = false;
      }

    private:

      void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
      {
        if(_paused)
        {
          gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
          _paused = false;
        }

        GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
        gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
        GstFlowReturn ret;

        g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
      }

     static void cb_newpad (GstElement *decodebin, GstPad *pad, 
                             gpointer data)
      {
        RosGstStream *client = reinterpret_cast<RosGstStream*>(data);

        GstCaps *caps;
        GstStructure *str;
        GstPad *audiopad;

        /* only link once */
        audiopad = gst_element_get_static_pad (client->_audio, "sink");
        if (GST_PAD_IS_LINKED (audiopad)) 
        {
          g_object_unref (audiopad);
          return;
        }

        /* check media type */
        caps = gst_pad_query_caps (pad, NULL);
        str = gst_caps_get_structure (caps, 0);
        if (!g_strrstr (gst_structure_get_name (str), "audio")) {
          gst_caps_unref (caps);
          gst_object_unref (audiopad);
          return;
        }

        gst_caps_unref (caps);

        /* link'n'play */
        gst_pad_link (pad, audiopad);

        g_object_unref (audiopad);
      }

     static void cb_need_data (GstElement *appsrc,
                   guint       unused_size,
                   gpointer    user_data)
     {
       ROS_WARN("need-data signal emitted! Pausing the pipeline");
       RosGstStream *client = reinterpret_cast<RosGstStream*>(user_data);
       gst_element_set_state(GST_ELEMENT(client->_pipeline), GST_STATE_PAUSED);
       client->_paused = true;
     }

      ros::NodeHandle _nh;
      ros::Subscriber _sub;
      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_audio, *_pay, *_sink, *_parser;
      GstElement *_playbin;
      GMainLoop *_loop;

      bool _paused;
  };
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_play");
  gst_init(&argc, &argv);

  audio_transport::RosGstStream client;

  ros::spin();
}
