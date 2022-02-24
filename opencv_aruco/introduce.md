姿态估计（Pose estimation）在 计算机视觉领域扮演着十分重要的角色： 机器人导航、增强现实以及其它。这一过程的基础是找到现实世界和图像投影之间的对应点。这通常是很困难的一步，因此我们常常用自己制作的或基本的Marker来让这一切变得更容易。    
最为流行的一个途径是基于二进制平方的标记。这种Marker的主要便利之处在于，一个Marker提供了足够多的对应（四个角）来获取相机的信息。同样的，内部的二进制编码使得算法非常健壮，允许应用错误检测和校正技术的可能性。   
aruco模块基于ArUco库，这是一个检测二进制marker的非常流行的库，是由Rafael Muñoz和Sergio Garrido完成的。  
aruco的函数包含在c++ #include <opencv2/aruco.hpp>   

# Marker和字典   
一个ArUco marker是一个二进制平方标记，它由一个宽的黑边和一个内部的二进制矩阵组成，内部的矩阵决定了它们的id。黑色的边界有利于快速检测到图像，二进制编码可以验证id，并且允许错误检测和矫正技术的应用。marker的大小决定了内部矩阵的大小。例如，一个4x4的marker由16bits组成。   
一些ArUco markers的例子：     
![img][V1]   
应当注意到，我们需要检测到一个Marker在空间中发生了旋转，但是，检测的过程需要确定它的初始角度，所以每个角落需要是明确的，不能有歧义，保证上述这点也是靠二进制编码完成的。      
markers的字典是在一个特殊应用中使用到的marker的集合。这仅仅是每个marker的二进制编码的链表。    
* 字典的主要性质是字典的大小和marker的大小：
  *  字典的大小是组成字典的marker的数量
  *  marker的大小是这些marker的尺寸（位的个数)
* aruco模块包含了一些预定义的字典，这些字典涵盖了一系列的字典大小和Marker尺寸。   

有些人可能会认为Marker的id是从十进制转成二进制的。但是，考虑到较大的marker会有较多的位数，管理如此多的数据不那么现实，这并不可能。反之，一个marker的id仅需是marker在它所在的字典的下标。例如，一个字典里的五个marker的id是：0,1,2,3和4。   
更多有关字典的信息在“选择字典”部分提及   

# 创建Marker  
在检测之前，我们需要打印marker，以把它们放到环境中。marker的图像可以使用drawMarker()函数生成。   
例如，让我们分析一下如下的调用：  
createMarker.cpp


[V1]:data:image/png;base64,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