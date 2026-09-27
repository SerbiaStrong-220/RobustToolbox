using System;
using System.Collections.Generic;
using System.IO; // ss220 add convertable nullable types
using System.Linq;
using System.Reflection;
using Robust.Shared.GameObjects; // ss220 add convertable nullable types
using Robust.Shared.IoC;
using Robust.Shared.Log;
using Robust.Shared.Prototypes;
using Robust.Shared.Reflection;
// ss220 add convertable nullable types start
using Robust.Shared.Serialization.Manager;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Value;
// ss220 add convertable nullable types end
using Robust.Shared.Utility;
using Robust.Shared.ViewVariables;
using YamlDotNet.RepresentationModel; // ss220 add convertable nullable types
using static Robust.Shared.ViewVariables.ViewVariablesBlobMembers;

namespace Robust.Server.ViewVariables.Traits
{
    internal sealed class ViewVariablesTraitMembers : ViewVariablesTrait
    {
        private readonly List<MemberInfo> _members = new();
        private readonly ISawmill _logger;

        public ViewVariablesTraitMembers(IViewVariablesSession session, ISawmill logger) : base(session)
        {
            _logger = logger;
        }

        public override ViewVariablesBlob? DataRequest(ViewVariablesRequest messageRequestMeta)
        {
            if (messageRequestMeta is ViewVariablesRequestMembers)
            {
                var members = new List<(MemberData mData, MemberInfo mInfo)>();
                var obj = Session.Object;
                var objType = Session.ObjectType;

                foreach (var property in objType.GetAllProperties())
                {
                    if (!ViewVariablesUtility.TryGetViewVariablesAccess(property, out var access))
                    {
                        continue;
                    }

                    if (!property.IsBasePropertyDefinition())
                    {
                        continue;
                    }

                    members.Add((new MemberData
                    {
                        Editable = access == VVAccess.ReadWrite,
                        Name = property.Name,
                        Type = property.PropertyType.AssemblyQualifiedName,
                        TypePretty = PrettyPrint.PrintUserFacingTypeShort(property.PropertyType, 2),
                        Value = property.GetValue(Session.Object),
                        PropertyIndex = _members.Count
                    }, property));
                    _members.Add(property);
                }

                foreach (var field in objType.GetAllFields())
                {
                    if (!ViewVariablesUtility.TryGetViewVariablesAccess(field, out var access))
                    {
                        continue;
                    }

                    members.Add((new MemberData
                    {
                        Editable = access == VVAccess.ReadWrite,
                        Name = field.Name,
                        Type = field.FieldType.AssemblyQualifiedName,
                        TypePretty = PrettyPrint.PrintUserFacingTypeShort(field.FieldType, 2),
                        Value = field.GetValue(obj),
                        PropertyIndex = _members.Count
                    }, field));

                    _members.Add(field);
                }

                foreach (var (mData, mInfo) in members)
                {
                    mData.Value = MakeValueNetSafe(mData.Value) ?? MakeNullValueNetSafe(mInfo.GetUnderlyingType());
                }

                var dataList = members
                    .OrderBy(p => p.mData.Name)
                    .GroupBy(p => p.mInfo.DeclaringType!)
                    .OrderByDescending(g => g.Key, TypeHelpers.TypeInheritanceComparer)
                    .Select(g =>
                    (
                        TypeAbbreviation.Abbreviate(g.Key),
                        g.Select(d => d.mData).ToList()
                    ))
                    .ToList();

                return new ViewVariablesBlobMembers
                {
                    MemberGroups = dataList
                };
            }

            if (messageRequestMeta is ViewVariablesRequestAllPrototypes protoReq)
            {
                var list = new List<string>();

                foreach (var prototype in IoCManager.Resolve<IPrototypeManager>().EnumeratePrototypes(protoReq.Variant))
                {
                    list.Add(prototype.ID);
                }

                return new ViewVariablesBlobAllPrototypes()
                {
                    Variant = protoReq.Variant,
                    Prototypes = list,
                };
            }

            return null;
        }

        public override bool TryGetRelativeObject(object property, out object? value)
        {
            if (property is not ViewVariablesMemberSelector selector)
            {
                return base.TryGetRelativeObject(property, out value);
            }

            if (selector.Index > _members.Count)
            {
                value = default;
                return false;
            }

            var member = _members[selector.Index];
            switch (member)
            {
                case PropertyInfo propertyInfo:
                    try
                    {
                        value = propertyInfo.GetValue(Session.Object);
                        return true;
                    }
                    catch (Exception e)
                    {
                        _logger.Error("Exception while getting property {0} on session {1} object {2}: {3}",
                            propertyInfo.Name, Session.SessionId, Session.Object, e);
                        value = default;
                        return false;
                    }

                case FieldInfo field:
                    try
                    {
                        value = field.GetValue(Session.Object);
                        return true;
                    }
                    catch (Exception e)
                    {
                        _logger.Error("Exception while modifying field {0} on session {1} object {2}: {3}",
                            field.Name, Session.SessionId, Session.Object, e);
                        value = default;
                        return false;
                    }

                default:
                    throw new InvalidOperationException();
            }
        }

        public override bool TryModifyProperty(object[] property, object value)
        {
            if (!(property[0] is ViewVariablesMemberSelector selector))
            {
                return base.TryModifyProperty(property, value);
            }

            if (selector.Index >= _members.Count)
            {
                return false;
            }

            var member = _members[selector.Index];

            switch (member)
            {
                case PropertyInfo propertyInfo:
                    try
                    {
                        // ss220 add convertable nullable types start
                        var converted = ConvertNullableValue(propertyInfo.PropertyType, value);
                        propertyInfo.GetSetMethod(true)!.Invoke(Session.Object, new[] {converted});
                        // ss220 add convertable nullable types end
                        return true;
                    }
                    catch (Exception e)
                    {
                        _logger.Error("Exception while modifying property {0} on session {1} object {2}: {3}",
                            propertyInfo.Name, Session.SessionId, Session.Object, e);
                        return false;
                    }

                case FieldInfo field:
                    try
                    {
                        // ss220 add convertable nullable types start
                        var converted = ConvertNullableValue(field.FieldType, value);
                        field.SetValue(Session.Object, converted);
                        // ss220 add convertable nullable types end
                        Session.ObjectChangeDelegate?.Invoke(Session.Object);

                        return true;
                    }
                    catch (Exception e)
                    {
                        _logger.Error("Exception while modifying field {0} on session {1} object {2}: {3}",
                            field.Name, Session.SessionId, Session.Object, e);
                        return false;
                    }

                default:
                    throw new InvalidOperationException();
            }
        }

        // ss220 add convertable nullable types start
        private object? ConvertNullableValue(Type type, object value)
        {
            // Only convert text sent for a Nullable<T> field or property.
            if (value is not string text || Nullable.GetUnderlyingType(type) == null)
                return value;

            using var reader = new StringReader(text);
            var yaml = new YamlStream();
            yaml.Load(reader);
            if (yaml.Documents.Count != 1)
                throw new ArgumentException("Expected one YAML value for a nullable VV member.");

            var node = yaml.Documents[0].RootNode.ToDataNode();
            if (node.IsNull)
                return null;

            if (node is ValueDataNode dataNode)
            {
                if (type == typeof(NetEntity?))
                    return NetEntity.Parse(dataNode.Value);

                if (type == typeof(EntityUid?))
                {
                    if (!IoCManager.Resolve<IEntityManager>().TryGetEntity(NetEntity.Parse(dataNode.Value), out var entity))
                        throw new ArgumentException("Unknown network entity id.");

                    return entity;
                }
            }

            // Convert other values to the declared type using the existing serializer.
            return IoCManager.Resolve<ISerializationManager>().Read(type, node);
        }
        // ss220 add convertable nullable types end
    }
}
