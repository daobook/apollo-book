# Apollo Auto 安装

参考：[开发者说｜Apollo 6.0 安装完全指南](https://mp.weixin.qq.com/s/mLJW29Eaq0O2JtgFaAqlXw)

前提：

1. Ubuntu20.04 系统
2. 安装 NVIDIA 显卡驱动（可参考）[nvidia-smi指令报错：Failed to initialize NVML: Driver解决 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/94378201)
3. 安装和配置 nvidia-docker2，可参考：[使用nvidia-docker2 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/37519492)


具体步骤如下：

1. 通过 SSH 方式或 HTTPS 方式克隆 Apollo 源码仓库：

```shell
# 使用 SSH 的方式
git clone git@github.com:ApolloAuto/apollo.git

# 使用 HTTPS 的方式
git clone https://github.com/ApolloAuto/apollo.git
```

GitHub 在国内访问速度可能很慢，可以使用：

```shell
# 使用 HTTPS 的方式
git clone https://github.com.cnpmjs.org/ApolloAuto/apollo.git
```

## 启动 Apollo Docker 开发容器

进入到 Apollo 源码根目录，打开终端，执行下述命令以启动 Apollo Docker 开发容器：

```shell
./docker/scripts/dev_start.sh
```

不出意外的话，启动成功后将得到下面信息：

```shell
Creating home directory `/home/znjs' ... Copying files from `/etc/skel' ...
[ OK ] Congratulations! You have successfully finished setting up Apollo Dev Environment.
[ OK ] To login into the newly created apollo_dev_znjs container, please run the following command:
[ OK ]   bash docker/scripts/dev_into.sh
[ OK ] Enjoy!
```

如果是在虚拟机中安装的 Ubuntu 或物理机没有配置 NVIDIA 显卡，但却又安装了 NVIDIA 驱动，则在执行上述启动容器的操作时将遇到报错，解决方法是直接卸载 NVIDIA 相关安装项：

<pre class="code-snippet__js" data-lang="nginx"><section><code><span class="code-snippet_outer"><span class="code-snippet__attribute">sudo</span> apt purge nvidia*</span></code></section></pre>

启动 Apollo Docker 开发容器后，执行下述命令进入容器：

```sh
./docker/scripts/dev_into.sh
```

![](images/install/1628651828823.png)

进入 Apollo Docker 开发容器后，在容器终端中执行下述命令构建 Apollo：

```shell
./apollo.sh build
```

如果报无权限创建目录的问题，在命令前加 `sudo` 即可。

## **启动 Apollo**

完成 Apollo 构建后，在容器终端中执行下述命令：

```shell
./scripts/bootstrap.sh start
```
