//***************************************************************************************************
// User Data
//  Copyright (c) Rylogic Ltd 2014
//***************************************************************************************************
// A container for user data.
// Structure is:
//    map<data-type-id, map<instance-id, IData*>>
//
// User data is lazily allocated when accessed.
// Usage:
//   // Allocates a default instance of 'MyThing' within the user data
//   m_user_data.get<MyThing>() = my_thing;
//
#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <new>

#if _MSC_VER < 1900
#  include <type_traits>
#  ifndef alignof
#    define alignof(T) std::alignment_of<T>::value
#  endif
#endif

namespace pr
{
	class UserData
	{
		// Ids associated with types
		using typeid_t = size_t;
		static typeid_t NextId()
		{
			static typeid_t id = 0;
			return id++;
		}
		template <typename TData> static typeid_t TypeId()
		{
			static typeid_t id = NextId();
			return id;
		}

		// Map from Id to data.
		struct IMap { virtual ~IMap() {} }; // Common base so that maps of unspecified types can be added to 'm_user_data'
		template <typename Id, typename Data> struct Map :IMap
		{
			// Aligned allocations
			template <typename T> static T* New()
			{
				auto p = _aligned_malloc(sizeof(T), alignof(T));
				return ::new (p) T;
			}
			template <typename T> static void Delete(T* obj)
			{
				obj->~T();
				return _aligned_free(obj);
			}

			// Use pointers to data, because 'Data' may not be copyable
			std::unordered_map<Id, Data*> m_cont;

			Map()
				:m_cont()
			{}
			~Map()
			{
				for (auto& x : m_cont)
					Delete<Data>(x.second);
			}

			// True if there are no contained items
			bool empty() const
			{
				return m_cont.empty();
			}

			// Return the number of items with 'm_id' == 'id'
			size_t count(Id id) const
			{
				return m_cont.count(id);
			}

			// Get a contained item by 'id'
			Data const& get(Id id) const
			{
				auto iter = m_cont.find(id);
				if (iter != std::end(m_cont)) return *iter->second;
				throw std::exception("user data not found");
			}
			Data& get(Id id, bool grow)
			{
				auto iter = m_cont.find(id);
				if (iter != std::end(m_cont)) return *iter->second;
				if (!grow) throw std::exception("user data not found");
				return *(m_cont[id] = New<Data>());
			}

			// Remove an item from the collection
			void erase(Id id)
			{
				auto iter = m_cont.find(id);
				if (iter == std::end(m_cont)) return;
				Delete<Data>(iter->second);
				m_cont.erase(iter);
			}
		};

		// A map from type Id to a map from instance Id to user data
		// E.g.
		//  auto   table = m_user_data[type-id];
		//  IData* data  = table[instance-id];
		// We allow the caller to choose the type of the instance-id. That is
		// why this type is not: Map<TypeId, Map<InstId, IData*>>
		std::vector<std::unique_ptr<IMap>> m_user_data;

	public:

		// True if this object contains user data of type 'TData' and associated with 'id'
		template <typename Data, typename InstId = int> bool has(InstId id = InstId()) const
		{
			auto map = find_map<InstId, Data>(TypeId<Data>());
			if (map == nullptr) return false;
			return map->count(id) != 0;
		}

		// Read access to user data of type 'TData'.
		// 'id' is an instance id. If not given, then the first instance is assumed.
		// 'id' can be any type used as a key to lookup data, e.g. Guid, or int typically.
		// e.g.  auto value = UserData.get<MyThing>(Guid(..)).Value;
		template <typename Data, typename InstId = int> Data const& get(InstId id = InstId()) const
		{
			auto map = find_map<InstId, Data>(TypeId<Data>());
			if (map == nullptr) throw std::exception("User data not found");
			return map->get(id);
		}

		// Write access to user data of type 'TData'. User data is lazily created.
		// 'id' is an instance id. If not given, then the first instance is assumed.
		// 'id' can be any type used as a key to lookup data, e.g. Guid, or int typically.
		// e.g.  UserData.get<MyThing>(Guid(..)) = my_thing;
		template <typename Data, typename InstId = int> Data& get(InstId id = InstId())
		{
			auto map = find_map<InstId, Data>(TypeId<Data>(), true);
			return map->get(id, true);
		}

		// Remove user data
		template <typename Data, typename InstId = int> void erase(InstId id = InstId())
		{
			auto map = find_map<InstId, Data>(TypeId<Data>(), false);
			if (map == nullptr) return;
			map->erase(id);
		}

	private:

		// Find the map associated with a type id
		template <typename InstId, typename Data> Map<InstId,Data> const* find_map(typeid_t type_id) const
		{
			return type_id < m_user_data.size() ? static_cast<Map<InstId,Data> const*>(m_user_data[type_id].get()) : nullptr;
		}
		template <typename InstId, typename Data> Map<InstId,Data>* find_map(typeid_t type_id, bool grow)
		{
			if (type_id >= m_user_data.size())
			{
				if (!grow) return nullptr;
				m_user_data.resize(type_id + 1);
				m_user_data[type_id].reset(new Map<InstId,Data>);
			}
			return static_cast<Map<InstId,Data>*>(m_user_data[type_id].get());
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/maths/maths.h"
namespace pr::common
{
	PRUnitTest(UserDataTests)
	{
		struct Blob
		{
			char str[20];
			Blob() :str() {}
			Blob(char const* s) :str() { strcat(str, s); }
		};

		Blob blob("HelloWorld\0");

		pr::UserData udtest;
		udtest.get<Blob>() = blob;
		udtest.get<double>() = 3.14;
		udtest.get<m4x4>(0) = m4x4::Identity(); // store aligned types

		PR_EXPECT(udtest.has<Blob>());
		PR_EXPECT(udtest.has<double>());
		PR_EXPECT(!udtest.has<int>());
		PR_EXPECT(udtest.has<m4x4>());

		PR_EXPECT(udtest.get<double>() == 3.14);
		PR_EXPECT(pr::str::Equal(udtest.get<Blob>().str, "HelloWorld"));
		PR_EXPECT(&udtest.get<Blob>() != &blob);
		PR_EXPECT(udtest.get<m4x4>() == m4x4::Identity());
		PR_EXPECT(&udtest.get<m4x4>() != &m4x4::Identity());

		udtest.get<double>() = 6.28;
		PR_EXPECT(udtest.get<double>() == 6.28);

		udtest.erase<double>();
		udtest.erase<Blob>();
		PR_EXPECT(!udtest.has<double>());
		PR_EXPECT(!udtest.has<Blob>());
	}
}
#endif