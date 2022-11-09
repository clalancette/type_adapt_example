# type_adapt_example

This package is meant to show off the performance benefits of using type adaptation when transferring data between two ROS nodes.

## Compilation

Compile like any ROS 2 package.
This has been tested on ROS 2 Humble.

## Test setup

For the rest of the discussion, we'll be assuming a two node setup where we have a single publisher and a single subscriber.

The publisher is publishing an empty 1920x1080, 3bpp image at approximately 30 Hz.
This emulates what an Intel Realsense D435i RGB camera would be outputting.

The subscriber is accepting the message, and doing exactly nothing with the data.

## Example 1: Separate processes, no type adaptation, no intra-process

This example is exemplified by the `image_pub_no_type_adapt` and `image_sub_no_type_adapt` executables.
The easiest way to run this would be to run the launch file that contains these two executables:

```
ros2 launch type_adapt_example image_no_type_adapt-launch.py
```

In this example, we launch two separate processes, one for the publisher and one for the subscriber.
For every image that is transmitted, roughly the following steps happen:

1.  The publisher allocates a `sensor_msgs::msg::Image` message, along with space for the data.
2.  The publisher fills out the data for the message.
3.  The publisher publishers the message, which ends up calling through the RMW stack, serializing the data, and delivering the data via UDP over localhost.
4.  The RMW layer on the subscriber receives the data via UDP over localhost, deserializes the data, and delivers it to the callback.

As can be seen, there is a lot of serializing, deserializing, and copying of data going on to deliver the data from one process to the next.
The result of this on a reference platform is that the publisher takes ~20% of a CPU to deliver the data, while the subscriber takes ~22% to receive the data.

Can we do better?

## Example 2: Single process composed, no type adaptation, no intra-process

The first thing we could do here would be to compose the two nodes into the same process.
This gives the middleware some chance to optimize the delivery of data, since it might be able to bypass sending through localhost completely if the publisher and subscriber are in the same process.

This example can be run with the following launch file:

```
ros2 launch type_adapt_example image_no_type_adapt-composed-launch.py
```

Indeed, by running the processes like this, we recognize a bunch of savings; the same data takes only ~20% to both send and receive.

Can we do better?

## Example 3: Separate processes, using type adaption, no intra-process

Another thing we can do is to try to involve type adaptation.
In this scenario, we break the publisher and subscriber into separate processes again, and then have each of them use type adaptation.
We are using type adapation between a `cv::Mat` and a `sensor_msgs::msg::Image`.
That means that the publisher and subscriber code both deal with a `cv::Mat`, but the underlying RMW layer handles a `sensor_msgs::msg::Image` (since it only knows how to deal with ROS types).
Conversion between the two is handled by `rclcpp` as appropriate.

This example can be run with the following launch file:

```
ros2 launch type_adapt_example image_type_adapt-launch.py
```

When we run this example, we see that it takes ~20% to do the publication, but ~60% to do the receiving.
This the worst result we've seen so far; what is happening?

By using type adaptation like this, we've actually only incurred overhead.
We still have to do all of the normal mechanisms of serializing and deserializing the message, but now we've also added a conversion to and from `cv::Mat`.
So it makes sense the the performance is worse.

Can we do better?

## Example 4: Single process composed, using type adaptation, no intra-process

This is the same as the previous example, but now we compose the nodes into a single process.
This example can be run with the following launch file:

```
ros2 launch type_adapt_example image_type_adapt-composed-launch.py
```

Like before, composing the processes together yields performance benefits.
Now it is only take ~60% CPU to both send and receive the data.
This is still worse than Example 2, however.

Can we do better?

## Example 5: Separate processes, using type adaptation, intra-process enabled

The next thing we can do is to enable intra-process communication.
This is a mechanism within `rclcpp` to bypass sending data to the RMW layer at all in certain situations.
In particular, both the publisher and subscriber need to be in the same process, and also need to share a context.
If those conditions are met, then data will be put into a queue directly in `rclcpp`, and will never be sent down to the RMW layer.

In this example, however, we are enabling intra-process, but the publisher and subscriber are in different processes.

This example can be run with the following launch file:

```
ros2 launch type_adapt_example image_type_adapt_intra-launch.py
```

When we run this example, we see that it takes ~20% to do the publication, but ~60% to do the receiving.
This makes sense; even though we attempted to enable intra-process, in reality these are separate processes so this is equivalent to Example 3.

Can we do better?

## Example 6: Single process composed, using type adaptation, intra-process enabled

As mentioned in the last section, if we have the publisher and subscriber in the same process and the same context, they can finally realize the full benefit of type adaptation and intra-process.

This example can be run with the following launch file:

```
ros2 launch type_adapt_example image_type_adapt_intra-composed-launch.py
```

When we run this example, it now only takes ~1% CPU to send and receive the data within the process.

To explore why a little further, consider what is happening inside of the process:

* The publisher is creating the `cv::Mat` message as a unique pointer, and calling `publish(std::move(message))` to transfer ownership.
* The `rclcpp` machinery knows that it has ownership of the pointer, that there is only a single subscriber, that the subscriber is in the same context, and that the subscriber also understands the `cv::Mat` type-adapted type.  Because it knows all of these things, it skips all processing and merely places the pointer into a queue and notifies the subscriber.
* The subscriber is woken up to do work and eventually call the callback.  It is told that the data is already in the queue.  Since the subscriber knows how to take the `cv::Mat` message directly, it doesn't need to do any conversion, and can just take the pointer directly and call the user callback with it.

## Coda

It's worth noting that not all combinations were explored above.
For instance, it might be worth seeing what happened with no type adaptation but intra-process in the same context.
That would likely yield similar performance benefits to Example 6.
