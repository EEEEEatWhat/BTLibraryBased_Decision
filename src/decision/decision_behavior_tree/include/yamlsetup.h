#ifndef YAMLSETUP_H
#define YAMLSETUP_H
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif