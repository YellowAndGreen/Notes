# Ubuntu安装docker

1. 完全清除之前的docker
2. 一键安装

```
wget –qO- https://get.docker.com/ |sh
```

3. 安装docker-compose

```
sudo apt-get install docker-compose
```

4. 可能会出现问题：**docker-credential-desktop not installed or not available in PATH**

​	解决方法：修改~/.docker/config.json，check config.json and replace "credsStore" by "credStore"

```yaml
{
  "stackOrchestrator" : "swarm",
  "experimental" : "disabled",
  "credStore" : "desktop"
}
```

# Overleaf docker使用

```text
docker pull sharelatex/sharelatex
```

从github上下载最新的[docker-compose.yml](https://link.zhihu.com/?target=https%3A//raw.githubusercontent.com/sharelatex/sharelatex/master/docker-compose.yml)文件并修改

文件地址：[https://github.com/overleaf/overleaf/blob/main/docker-compose.yml](https://link.zhihu.com/?target=https%3A//github.com/overleaf/overleaf/blob/main/docker-compose.yml)

放到工作目录/home/sharelatex中，并用vim进行修改，主要是**修改sharelatex映射端口，和挂载**



**主要修改内容：**

ports:

\- 80:80 #修改成自己准备在服务器上开放的端口，如**5000**:80,不要和其他应用端口冲突

volumes:

\- ~/sharelatex_data:/var/lib/sharelatex

\#修改成工作目录**/home/xu/sharelatex/sharelatex_data**:/var/lib/sharelatex

volumes:

\- ~/mongo_data:/data/db #修改成工作目录**/home/xu/sharelatex/mongo_data**:/data/db

volumes:

\- ~/redis_data:/data #修改成工作目录**/home/xu/sharelatex/redis_data**:/data

**可选：**

增加**挂载texlive包文件夹**如下，可以把texlive完整包挂载到容器外，为update后不从新下载包文件准备！

```text
 volumes:
            - ~/sharelatex_data:/var/lib/sharelatex
            - ~/sharelatex_texlive://usr/local/texlive
            - /var/run/docker.sock:/var/run/docker.sock
            #- /var/clsi/compiles:/var/www/sharelatex/clsi/compiles
```

```
docker-compose up
```



升级*TeXLive*

```sql
docker exec sharelatex tlmgr install scheme-full
```

使用*tlmgr*升级*TeXLive*会有以下报错

```sql
tlmgr: Remote repository is newer than local (2017 < 2018)



Cross release updates are only supported with



  update-tlmgr-latest(.sh/.exe) --update



Please see https://tug.org/texlive/upgrade.html for details.
```

根据提示去寻找*update-tlmgr-latest.sh*进行配置。

```crystal
root@056c250ddb2a:/# wget http://mirror.ctan.org/systems/texlive/tlnet/update-tlmgr-latest.sh
sh update-tlmgr-latest.sh
root@056c250ddb2a:/# tlmgr option repository http://mirrors.ustc.edu.cn/CTAN/systems/texlive/tlnet
root@056c250ddb2a:/# tlmgr install scheme-full
```

**九、**账户设置

```text
docker exec sharelatex /bin/bash -c "cd /var/www/sharelatex; grunt user:create-admin --email=602344037@qq.com"
```

